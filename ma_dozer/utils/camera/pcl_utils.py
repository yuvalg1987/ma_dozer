import numpy as np
import cupy as cp
import cv2
import sys
import math
import cupyx.scipy.ndimage.filters as filters
from cupyx.time import repeat

import utils.general_config as general_config


###################
## CPU Functions ##
###################

def create_xyz(depth_image, camera_intrinsics):
    image_height, image_width = depth_image.shape
    x_grid, y_grid = np.meshgrid(np.arange(0, image_width), np.arange(0, image_height))

    vertices_c = np.zeros((3, image_height * image_width), dtype=np.float32)
    vertices_c[2, :] = depth_image.reshape(-1)
    vertices_c[0, :] = ((x_grid.reshape(-1) - camera_intrinsics[0, 2]) / camera_intrinsics[0, 0]) * vertices_c[2, :]
    vertices_c[1, :] = ((y_grid.reshape(-1) - camera_intrinsics[1, 2]) / camera_intrinsics[1, 1]) * vertices_c[2, :]

    return vertices_c


def orthographic_projection(pcl, pixel_density, lower_bound, upper_bound, grid_width, grid_height):
    reduced_pcl = pcl[np.all(pcl[:, :3] >= lower_bound, axis=1) & np.all(pcl[:, :3] <= upper_bound, axis=1), :]

    xy_image = np.zeros((grid_height, grid_width, pcl.shape[1]))
    ref = 0

    xy_image[:, :, 2] = np.full((grid_height, grid_width), lower_bound[2] - 1e-2)
    xx, yy = np.meshgrid(range(grid_width), range(grid_height), indexing="xy")

    xy_image[:, :, 0] = lower_bound[0] + np.array(1 + 2 * xx, dtype=np.float) / (2 * pixel_density)
    xy_image[:, :, 1] = lower_bound[1] + np.array(1 + 2 * yy, dtype=np.float) / (2 * pixel_density)

    pixels = np.array(((reduced_pcl[:, :2] - lower_bound[:2]) * pixel_density), dtype=np.int)

    xy_image[pixels[:, 1], pixels[:, 0], 2:] = reduced_pcl[:, 2:]

    depth = xy_image[:, :, 2]
    xy_image[:, :, 2] = depth

    return xy_image[:, :, 2], xy_image[:, :, 3:].astype(np.uint8)


def transform_vertices(vertices_i, rot_i2j, t_j2i_j):
    vertices_j = rot_i2j @ vertices_i + t_j2i_j
    return vertices_j


#############################
## RGBD to PCL in World CS ##
#############################

depth_to_pcl_world_kernel = cp.RawKernel(r'''
extern "C" __global__
void depth_to_pcl_world_kernel(const float* Z_c, int image_width, int image_height, 
                               const float* intrinsics, int intrinsics_step,
                               const float* rot_c2w   , int rot_c2w_step,
                               const float* t_w2c_w   , int t_w2c_w_step,
                               float* X_w, float* Y_w, float* Z_w ) {
 	int global_idx_x = blockIdx.x * blockDim.x + threadIdx.x;

    int pixel_num = image_width * image_height;
	if (global_idx_x >= pixel_num)
		return;
	int curr_row_idx = global_idx_x / image_width;
	int curr_col_idx = global_idx_x % image_width;

    float x_local   = (float)curr_col_idx;
    float y_local   = (float)curr_row_idx;
    float Z_c_local = Z_c[global_idx_x];
    float f_x = intrinsics[0 * intrinsics_step + 0];
    float f_y = intrinsics[1 * intrinsics_step + 1];
    float c_x = intrinsics[0 * intrinsics_step + 2];
    float c_y = intrinsics[1 * intrinsics_step + 2];
    float X_c = Z_c_local * ((x_local - c_x) / f_x);
    float Y_c = Z_c_local * ((y_local - c_y) / f_y);
    X_w[global_idx_x] = rot_c2w[0] * X_c + rot_c2w[1] * Y_c + rot_c2w[2] * Z_c_local + t_w2c_w[0];
    Y_w[global_idx_x] = rot_c2w[3] * X_c + rot_c2w[4] * Y_c + rot_c2w[5] * Z_c_local + t_w2c_w[1];
    Z_w[global_idx_x] = rot_c2w[6] * X_c + rot_c2w[7] * Y_c + rot_c2w[8] * Z_c_local + t_w2c_w[2];
}
''', 'depth_to_pcl_world_kernel')


def depth_to_pcl_world(depth_image_d, intrinsics_d, rot_c2w_d, t_w2c_w_d, pcl_xyzbgr_d):
    image_height, image_width = depth_image_d.shape
    pixel_num = image_width * image_height

    threads_per_block = 64
    block_num = math.ceil((pixel_num + threads_per_block - 1) / threads_per_block)

    intrinsics_step = int(intrinsics_d.strides[0] / intrinsics_d.strides[1])
    rot_c2w_d_step = int(rot_c2w_d.strides[0] / rot_c2w_d.strides[1])
    t_w2c_w_d_step = int(t_w2c_w_d.strides[0] / t_w2c_w_d.strides[0])

    depth_to_pcl_world_kernel((block_num,), (threads_per_block,),
                              (depth_image_d.reshape(-1), image_width, image_height,
                               intrinsics_d, intrinsics_step,
                               rot_c2w_d, rot_c2w_d_step,
                               t_w2c_w_d, t_w2c_w_d_step,
                               pcl_xyzbgr_d[0, :], pcl_xyzbgr_d[1, :], pcl_xyzbgr_d[2, :]))


def depth_to_pcl_world_tester(depth_image_h, intrinsics_h, rot_c2w_h, t_w2c_w_h):
    depth_image_d = cp.asarray(depth_image_h, dtype=cp.float32)
    intrinsics_d = cp.asarray(intrinsics_h, dtype=cp.float32)
    rot_c2w_d = cp.asarray(rot_c2w_h, dtype=cp.float32)
    t_w2c_w_d = cp.asarray(t_w2c_w_h, dtype=cp.float32)

    image_height, image_width = depth_image_d.shape

    #########
    ## Ref ##
    #########

    vertices_c_ref_h = create_xyz(depth_image_h, intrinsics_h)
    vertices_w_ref_h = transform_vertices(vertices_c_ref_h, rot_c2w_h, t_w2c_w_h)

    ##########
    ## Test ##
    ##########

    vertices_w_test_d = cp.zeros((3, image_height * image_width), dtype=np.float32)
    pcl_xyzbgr_w_d = cp.zeros((6, image_height * image_width), dtype=np.float32)

    depth_to_pcl_world(depth_image_d, intrinsics_d, rot_c2w_d, t_w2c_w_d, vertices_w_test_d)
    depth_to_pcl_world(depth_image_d, intrinsics_d, rot_c2w_d, t_w2c_w_d, pcl_xyzbgr_w_d)

    #############
    ## Compare ##
    #############

    vertices_w_test_h = cp.asnumpy(vertices_w_test_d)
    print(np.allclose(vertices_w_test_h, vertices_w_ref_h, atol=0.001))

    vertices_w_test_h = cp.asnumpy(pcl_xyzbgr_w_d[:3, :])
    print(np.allclose(vertices_w_test_h, vertices_w_ref_h, atol=0.001))


#############################
## Flatten PCL to XY Plane ##
#############################

project_pcl_to_xy_kernel = cp.RawKernel(r'''
extern "C" __global__
void project_pcl_to_xy_kernel(const float* pcl_xyzbgr , int pcl_xyzbgr_step,
                                    float* pcl_flat   , int pcl_flat_step, 
                                    int*   pcl_norm_counter,
                                    const float* lower_bound, const float* upper_bound, float pixel_density,
                                    int pixel_num, int grid_width, int grid_height) {
	int global_idx_x = blockIdx.x * blockDim.x + threadIdx.x;
	if (global_idx_x >= pixel_num)
		return;

    float currX = pcl_xyzbgr[global_idx_x + 0 * pcl_xyzbgr_step];
    float currY = pcl_xyzbgr[global_idx_x + 1 * pcl_xyzbgr_step];
    float currZ = pcl_xyzbgr[global_idx_x + 2 * pcl_xyzbgr_step];
    float currB = pcl_xyzbgr[global_idx_x + 3 * pcl_xyzbgr_step];
    float currG = pcl_xyzbgr[global_idx_x + 4 * pcl_xyzbgr_step];
    float currR = pcl_xyzbgr[global_idx_x + 5 * pcl_xyzbgr_step];

    bool pcl_bbox =  (currX > lower_bound[0]) && (currX < upper_bound[0]) &&
                     (currY > lower_bound[1]) && (currY < upper_bound[1]) &&
                     (currZ > lower_bound[2]) && (currZ < upper_bound[2]);
    int grid_pix_idx_x = int( (currX - lower_bound[0]) * pixel_density);
    int grid_pix_idx_y = int( (currY - lower_bound[1]) * pixel_density);   
    if (grid_pix_idx_x >= 0 && grid_pix_idx_x < grid_width &&
        grid_pix_idx_y >= 0 && grid_pix_idx_y < grid_height &&
        pcl_bbox == true) {
        atomicAdd(&pcl_flat[grid_pix_idx_y * grid_width + grid_pix_idx_x + 2 * pcl_flat_step], currZ);
        atomicAdd(&pcl_flat[grid_pix_idx_y * grid_width + grid_pix_idx_x + 3 * pcl_flat_step], currB);
        atomicAdd(&pcl_flat[grid_pix_idx_y * grid_width + grid_pix_idx_x + 4 * pcl_flat_step], currG);
        atomicAdd(&pcl_flat[grid_pix_idx_y * grid_width + grid_pix_idx_x + 5 * pcl_flat_step], currR);
        atomicAdd(&pcl_norm_counter[grid_pix_idx_y * grid_width + grid_pix_idx_x], 1);

    }

}
''', 'project_pcl_to_xy_kernel')

div_kernel = cp.RawKernel(r'''
extern "C" __global__
void div_kernel(const int* denom , int denom_step,
                      float* nom   , int nom_step, 
                      int elem_num) {
	int global_idx_x = blockIdx.x * blockDim.x + threadIdx.x;
	if (global_idx_x >= elem_num)
		return;

    int curr_denom = denom[global_idx_x];

    if (curr_denom > 0) {
        nom[global_idx_x + 0 * nom_step] = nom[global_idx_x + 0 * nom_step] / curr_denom;
        nom[global_idx_x + 1 * nom_step] = nom[global_idx_x + 1 * nom_step] / curr_denom;
        nom[global_idx_x + 2 * nom_step] = nom[global_idx_x + 2 * nom_step] / curr_denom;
        nom[global_idx_x + 3 * nom_step] = nom[global_idx_x + 3 * nom_step] / curr_denom;
        nom[global_idx_x + 4 * nom_step] = nom[global_idx_x + 4 * nom_step] / curr_denom;
        nom[global_idx_x + 5 * nom_step] = nom[global_idx_x + 5 * nom_step] / curr_denom;
    }

}
''', 'div_kernel')

init_flat_pcl_kernel = cp.RawKernel(r'''
extern "C" __global__
    void init_flat_pcl(const float* lower_bound_d, const float* upper_bound_d,
                       int grid_width, int grid_height, float pixel_density,
                       float* pcl_xyzbgr, int pcl_xyzbgr_step) {

    int global_idx_x = blockIdx.x * blockDim.x + threadIdx.x;
    int pixel_num = grid_width * grid_height;

	if (global_idx_x >= pixel_num)
		return;
	int curr_row_idx = global_idx_x / grid_width;
	int curr_col_idx = global_idx_x % grid_height;

    pcl_xyzbgr[global_idx_x + 0 * pcl_xyzbgr_step] = lower_bound_d[0] + (1 + 2 * curr_col_idx) / (2 * pixel_density);
    pcl_xyzbgr[global_idx_x + 1 * pcl_xyzbgr_step] = lower_bound_d[1] + (1 + 2 * curr_row_idx) / (2 * pixel_density);
}
''', 'init_flat_pcl')


def project_pcl_to_xy(pcl_xyzbgr_d,
                      lower_bound_d, upper_bound_d, pixel_density,
                      grid_width, grid_height):
    pixel_num = pcl_xyzbgr_d.shape[1]
    grid_num = grid_width * grid_height

    threads_per_block = 64
    block_num = math.ceil((pixel_num + threads_per_block - 1) / threads_per_block)

    pcl_flat_d = cp.zeros((6, grid_num), dtype=np.float32)
    pcl_norm_counter_d = cp.zeros((1, grid_num), dtype=np.int32)

    pcl_xyzbgr_step = int(pcl_xyzbgr_d.strides[0] / pcl_xyzbgr_d.strides[1])
    pcl_flat_step = int(pcl_flat_d.strides[0] / pcl_flat_d.strides[1])
    pcl_norm_counter_step = int(pcl_norm_counter_d.strides[0] / pcl_norm_counter_d.strides[1])

    # init_flat_pcl_kernel((block_num,), (threads_per_block,),
    #                      (lower_bound_d, upper_bound_d,
    #                      grid_width, grid_height, pixel_density,
    #                     pcl_flat_d, pcl_flat_step))

    project_pcl_to_xy_kernel((block_num,), (threads_per_block,),
                             (pcl_xyzbgr_d, pcl_xyzbgr_step,
                              pcl_flat_d, pcl_flat_step,
                              pcl_norm_counter_d,
                              lower_bound_d, upper_bound_d, pixel_density,
                              pixel_num, grid_width, grid_height))

    block_num = math.ceil((grid_num + threads_per_block - 1) / threads_per_block)

    div_kernel((block_num,), (threads_per_block,),
               (pcl_norm_counter_d, pcl_norm_counter_step,
                pcl_flat_d, pcl_flat_step, grid_num))

    height_map_d = pcl_flat_d[2, :].reshape((grid_height, grid_width))
    topview_image_d = pcl_flat_d[3:, :].T.reshape((grid_height, grid_width, 3)).astype(np.uint8)

    topview_image_b_d = pcl_flat_d[3, :].reshape((grid_height, grid_width))
    topview_image_g_d = pcl_flat_d[4, :].reshape((grid_height, grid_width))
    topview_image_r_d = pcl_flat_d[5, :].reshape((grid_height, grid_width))

    topview_image_blurred_b_d = filters.median_filter(topview_image_b_d, 3).astype(np.uint8)
    topview_image_blurred_g_d = filters.median_filter(topview_image_g_d, 3).astype(np.uint8)
    topview_image_blurred_r_d = filters.median_filter(topview_image_r_d, 3).astype(np.uint8)

    topview_image_blurred_d = cp.stack(
        (topview_image_blurred_b_d, topview_image_blurred_g_d, topview_image_blurred_r_d), 2)
    topview_image_d = cp.where(height_map_d[:, :, cp.newaxis] == 0, topview_image_blurred_d, topview_image_d)

    # height_map_blurred_d = filters.median_filter(height_map_d, 10)
    height_map_blurred_d = height_map_d

    height_map_d = cp.where(height_map_d == 0, height_map_blurred_d, height_map_d)

    height_map_h = cp.asnumpy(height_map_d)
    topview_image_h = cp.asnumpy(topview_image_d)

    return height_map_h, topview_image_h


def project_pcl_to_xy_tester(color_image_h, depth_image_h,
                             intrinsics_h, rot_c2w_h, t_w2c_w_h):
    depth_image_d = cp.asarray(depth_image_h, dtype=cp.float32)

    image_height, image_width = depth_image_d.shape

    #########
    ## Ref ##
    #########

    vertices_c_ref_h = create_xyz(depth_image_h, intrinsics_h)
    vertices_w_ref_h = transform_vertices(vertices_c_ref_h, rot_c2w_h, t_w2c_w_h)

    depth_image_d = cp.asarray(depth_image_h, dtype=np.float32)
    color_image_d = cp.asarray(color_image_h, dtype=np.float32)
    intrinsics_d = cp.asarray(intrinsics_h, dtype=np.float32)
    rot_c2w_d = cp.asarray(rot_c2w_h, dtype=np.float32)
    t_w2c_w_d = cp.asarray(t_w2c_w_h, dtype=np.float32)

    pcl_xyzbgr_test_d = cp.zeros((6, image_height * image_width), dtype=np.float32)
    pcl_xyzbgr_test_d[3, :] = color_image_d[:, :, 0].flatten()
    pcl_xyzbgr_test_d[4, :] = color_image_d[:, :, 1].flatten()
    pcl_xyzbgr_test_d[5, :] = color_image_d[:, :, 2].flatten()

    depth_to_pcl_world(depth_image_d, intrinsics_d, rot_c2w_d, t_w2c_w_d, pcl_xyzbgr_test_d)

    pcl_xyzbgr_test_h = cp.asnumpy(pcl_xyzbgr_test_d)
    print(np.allclose(pcl_xyzbgr_test_h[0:3, :], vertices_w_ref_h, atol=0.001))

    pcl_xyzbgr_h = np.zeros((6, image_height * image_width), dtype=np.float32)
    pcl_xyzbgr_h[:3, :] = vertices_w_ref_h
    pcl_xyzbgr_h[3, :] = color_image_h[:, :, 0].flatten()
    pcl_xyzbgr_h[4, :] = color_image_h[:, :, 1].flatten()
    pcl_xyzbgr_h[5, :] = color_image_h[:, :, 2].flatten()

    lower_bound, upper_bound, grid_width, grid_height = calc_bbox(pcl_xyzbgr_h,
                                                                  low_p=general_config.bbox_low_percentile,
                                                                  high_p=general_config.bbox_high_percentile,
                                                                  pixel_density=general_config.pixel_density)
    pixel_density = np.float32(2.1)
    pcl_flat_ref_h = orthographic_projection(pcl_xyzbgr_h.T, pixel_density, lower_bound, upper_bound, grid_width,
                                             grid_height)

    ##########
    ## Test ##
    ##########

    pcl_xyzbgr_d = cp.asarray(pcl_xyzbgr_h, dtype=np.float32)
    lower_bound_d = cp.asarray(lower_bound, dtype=np.float32)
    upper_bound_d = cp.asarray(upper_bound, dtype=np.float32)

    height_map_h, topview_image_h = project_pcl_to_xy(pcl_xyzbgr_d,
                                                      lower_bound_d, upper_bound_d, pixel_density,
                                                      grid_width, grid_height)

    height_map_h, topview_image_h = project_pcl_to_xy(pcl_xyzbgr_test_d,
                                                      lower_bound_d, upper_bound_d, pixel_density,
                                                      grid_width, grid_height)


####################
## Interface Func ##
####################

def calc_bbox(pcl, low_p=3, high_p=97, pixel_density=2.1):
    pcl_min = np.percentile(pcl[:3, :], low_p, axis=1)
    pcl_max = np.percentile(pcl[:3, :], high_p, axis=1)

    bbox_center = (pcl_min + pcl_max) / 2
    bbox_half_sizes = (pcl_max - pcl_min) / 2

    n_x = int(np.ceil(bbox_half_sizes[0] * pixel_density))
    if n_x % 2 == 1:
        n_x += 1

    n_y = int(np.ceil(bbox_half_sizes[1] * pixel_density))
    if n_y % 2 == 1:
        n_y += 1

    n_z = int(np.ceil(bbox_half_sizes[2] * pixel_density))
    if n_z % 2 == 1:
        n_z += 1

    n = np.array([n_x, n_y, n_z], dtype=np.float)
    grid_width = 2 * n_x
    grid_height = 2 * n_y
    lower_bound = bbox_center - n / pixel_density
    upper_bound = bbox_center + n / pixel_density

    return lower_bound, upper_bound, grid_width, grid_height


def rgbd_to_top(color_image_h, depth_image_h,
                intrinsics_d, rot_c2w_d, t_w2c_w_d,
                lower_bound_d, upper_bound_d, pixel_density,
                grid_width, grid_height):
    depth_image_d = cp.asarray(depth_image_h, dtype=np.float32)
    color_image_d = cp.asarray(color_image_h, dtype=np.float32)

    image_height, image_width = depth_image_d.shape

    pcl_xyzbgr_w_d = cp.zeros((6, image_height * image_width), dtype=np.float32)
    pcl_xyzbgr_w_d[3, :] = color_image_d[:, :, 0].flatten()
    pcl_xyzbgr_w_d[4, :] = color_image_d[:, :, 1].flatten()
    pcl_xyzbgr_w_d[5, :] = color_image_d[:, :, 2].flatten()

    depth_to_pcl_world(depth_image_d, intrinsics_d, rot_c2w_d, t_w2c_w_d, pcl_xyzbgr_w_d)

    height_map_h, topview_image_h = project_pcl_to_xy(pcl_xyzbgr_w_d,
                                                      lower_bound_d, upper_bound_d, pixel_density,
                                                      grid_width, grid_height)

    return height_map_h, topview_image_h


def allocation_perf(color_image_h, depth_image_h):
    depth_image_d = cp.asarray(depth_image_h, dtype=np.float32)
    color_image_d = cp.asarray(color_image_h, dtype=np.float32)

    image_height, image_width = depth_image_d.shape

    pcl_xyzbgr_w_d = cp.zeros((6, image_height * image_width), dtype=np.float32)
    pcl_xyzbgr_w_d[3, :] = color_image_d[:, :, 0].flatten()
    pcl_xyzbgr_w_d[4, :] = color_image_d[:, :, 1].flatten()
    pcl_xyzbgr_w_d[5, :] = color_image_d[:, :, 2].flatten()


def rgbd_to_top_tester(color_image_h, depth_image_h,
                       intrinsics_h, rot_c2w_h, t_w2c_w_h, pixel_density):
    intrinsics_d = cp.asarray(intrinsics_h, dtype=cp.float32)
    rot_c2w_d = cp.asarray(rot_c2w_h, dtype=cp.float32)
    t_w2c_w_d = cp.asarray(t_w2c_w_h, dtype=cp.float32)

    image_height, image_width = depth_image_h.shape

    vertices_c_ref_h = create_xyz(depth_image_h, intrinsics_h)
    vertices_w_ref_h = transform_vertices(vertices_c_ref_h, rot_c2w_h, t_w2c_w_h)

    pcl_xyzbgr_h = np.zeros((6, image_height * image_width), dtype=np.float32)
    pcl_xyzbgr_h[:3, :] = vertices_w_ref_h
    pcl_xyzbgr_h[3, :] = color_image_h[:, :, 0].flatten()
    pcl_xyzbgr_h[4, :] = color_image_h[:, :, 1].flatten()
    pcl_xyzbgr_h[5, :] = color_image_h[:, :, 2].flatten()

    lower_bound, upper_bound, grid_width, grid_height = calc_bbox(pcl_xyzbgr_h,
                                                                  low_p=general_config.bbox_low_percentile,
                                                                  high_p=general_config.bbox_high_percentile,
                                                                  pixel_density=general_config.pixel_density)
    lower_bound_d = cp.asarray(lower_bound, dtype=np.float32)
    upper_bound_d = cp.asarray(upper_bound, dtype=np.float32)

    height_map_h, topview_image_h = rgbd_to_top(color_image_h, depth_image_h,
                                                intrinsics_d, rot_c2w_d, t_w2c_w_d,
                                                lower_bound_d, upper_bound_d, pixel_density,
                                                grid_width, grid_height)


def test_performence(color_image_h, depth_image_h,
                     intrinsics_h, rot_c2w_h, t_w2c_w_h):
    color_image_d = cp.asarray(color_image_h, dtype=cp.float32)
    depth_image_d = cp.asarray(depth_image_h, dtype=cp.float32)
    intrinsics_d = cp.asarray(intrinsics_h, dtype=cp.float32)
    rot_c2w_d = cp.asarray(rot_c2w_h, dtype=cp.float32)
    t_w2c_w_d = cp.asarray(t_w2c_w_h, dtype=cp.float32)

    image_height, image_width, channel_num = color_image_d.shape
    pixel_num = image_height * image_width

    ####################
    ## Depth to World ##
    ####################

    pcl_xyzbgr_w_d = cp.zeros((6, pixel_num), dtype=np.float32)

    print(repeat(depth_to_pcl_world, (depth_image_d, intrinsics_d, rot_c2w_d, t_w2c_w_d, pcl_xyzbgr_w_d), n_repeat=200))

    #################
    ## Flatten PCL ##
    #################

    pcl_xyzbgr_w_h = cp.asnumpy(pcl_xyzbgr_w_d)
    lower_bound, upper_bound, grid_width, grid_height = calc_bbox(pcl_xyzbgr_w_h, low_p=5, high_p=95, pixel_density=2.1)

    pixel_density = np.float32(2.1)
    lower_bound_d = cp.asarray(lower_bound, dtype=np.float32)
    upper_bound_d = cp.asarray(upper_bound, dtype=np.float32)

    print(repeat(project_pcl_to_xy, (pcl_xyzbgr_w_d,
                                     lower_bound_d, upper_bound_d, pixel_density,
                                     grid_width, grid_height), n_repeat=200))

    #################
    ## Alllocation ##
    #################
    mempool = cp.get_default_memory_pool()
    pinned_mempool = cp.get_default_pinned_memory_pool()

    allocation_perf(color_image_h, depth_image_h)

    print(mempool.used_bytes())  # 0
    print(mempool.total_bytes())  # 0
    print(pinned_mempool.n_free_blocks())

    allocation_perf(color_image_h, depth_image_h)

    print(mempool.used_bytes())  # 0
    print(mempool.total_bytes())  # 0
    print(pinned_mempool.n_free_blocks())

    print(repeat(allocation_perf, (color_image_h, depth_image_h), n_repeat=200))

    #########
    ## All ##
    #########

    print(repeat(rgbd_to_top, (color_image_h, depth_image_h,
                               intrinsics_d, rot_c2w_d, t_w2c_w_d,
                               lower_bound_d, upper_bound_d, pixel_density,
                               grid_width, grid_height), n_repeat=200))


def main():
    color_image_h = np.load('../camera_calibration/initial_color_image.npy').astype(np.float32)
    depth_image_h = np.load('../camera_calibration/initial_depth_image.npy').astype(np.float32)
    intrinsics_h = np.load('../camera_calibration/intrinsics_params.npy').astype(np.float32)
    rot_c2w_h = np.load('../camera_calibration/rot_c2w.npy').astype(np.float32)
    t_w2c_w_h = np.load('../camera_calibration/t_w2c_w.npy').astype(np.float32)

    mempool = cp.get_default_memory_pool()
    pinned_mempool = cp.get_default_pinned_memory_pool()

    allocation_perf(color_image_h, depth_image_h)

    print(mempool.used_bytes())  # 0
    print(mempool.total_bytes())  # 0
    print(pinned_mempool.n_free_blocks())

    allocation_perf(color_image_h, depth_image_h)

    print(mempool.used_bytes())  # 0
    print(mempool.total_bytes())  # 0
    print(pinned_mempool.n_free_blocks())

    depth_to_pcl_world_tester(depth_image_h, intrinsics_h, rot_c2w_h, t_w2c_w_h)

    project_pcl_to_xy_tester(color_image_h, depth_image_h,
                             intrinsics_h, rot_c2w_h, t_w2c_w_h)

    rgbd_to_top_tester(color_image_h, depth_image_h,
                       intrinsics_h, rot_c2w_h, t_w2c_w_h, general_config.pixel_density)

    test_performence(color_image_h, depth_image_h,
                     intrinsics_h, rot_c2w_h, t_w2c_w_h)