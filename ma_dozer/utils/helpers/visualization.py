try:
    from matplotlib import pyplot as plt
except:
    import matplotlib as mpl
    mpl.use('Agg')
    from matplotlib import pyplot as plt
import cv2
import numpy as np
import matplotlib as mpl
from typing import List

from dozer_envs.scripts.state.state_classes import State, Raster, Position
from dozer_envs.utils.general.cs_utils import rotate_points_from_body_to_inertial
from dozer_envs.utils.general.dcm_utils import translate_points
from dozer_prototype.scripts.helpers.state import RealDozerPose, RealDozerAction

def plot_heightmap_with_contours(heightmap_raster: Raster,
                                 title: str = "height map",
                                 num_contours: int = 10,
                                 with_contour_labels: bool = False,
                                 fig=None, ax=None, add_colorbar=True,
                                 plot_contours=True, cax: plt.Axes = None,
                                 normalize_colors: bool = True,
                                 levels: np.ndarray = None, units: str = 'CM'):
    """
    this function plots a 2D graph with the height map and contours of height levels
    :param heightmap_raster:
    :param normalize_colors: should use specific normalization or general one
    :param title: title of figure (string)
    :param num_contours: number of contours to plot on top of height map
    :param with_contour_labels: if True will plot height labels
    :param fig: handler for figure to use (if None create here)
    :param ax: handler for axis to use (if None create here)
    :param add_colorbar: if True colorbar will be added to the plot
    :param plot_contours: if True will plot contours as well as regular plot
    :param cax: handler for colorbar axis (if None will create)
    :param levels: value where the contour will be drawn
    :param units: units of axis [m / cm] taken from raster_res

    :return:
    """
    # convert height map values from pixels to meters

    heightmap = heightmap_raster.raster_data * heightmap_raster.raster_res.height

    if fig is None:
        fig, ax = plt.subplots()
        if add_colorbar:
            div = make_axes_locatable(ax)
            cax = div.append_axes('right', '5%', '5%')
            cax.clear()
    min_x = 0
    max_x = heightmap_raster.area_size.width
    min_y = 0
    max_y = heightmap_raster.area_size.length
    max_z = np.nanmax(heightmap)
    min_z = np.nanmin(heightmap)
    xx, yy = create_meshgrid(area_size=heightmap_raster.area_size, raster_res=heightmap_raster.raster_res)

    if plot_contours:
        if levels is None:
            val1 = heightmap.min()
            val2 = heightmap.min() * 0.95
            if heightmap.min() < 0:
                levels = np.array((val1, val2), dtype=np.float32)
            else:
                levels = np.array((val2, val1), dtype=np.float32)
        contours = ax.contour(xx, yy, heightmap, levels=levels, colors='black')
    else:
        contours = None
    if plot_contours and with_contour_labels:
        ax.clabel(contours, inline=True, fontsize=8)
    cmap = plt.get_cmap('YlOrBr')

    if max_z <= 0.3:
        max_z = 0.3 + EPSILON
    if min_z >= 0.3:
        min_z = 0.3 - EPSILON
    # if max_z < 1:
    #     max_z = 2
    if normalize_colors:
        norm = colors.TwoSlopeNorm(vmin=min_z, vcenter=0.3, vmax=max_z)
    else:
        norm = colors.TwoSlopeNorm(vmin=np.nanmin(heightmap).item() - EPSILON, vcenter=np.nanmedian(heightmap).item(),
                                   vmax=np.nanmax(heightmap).item() + EPSILON)
    patch = ax.imshow(heightmap, extent=[min_x, max_x, min_y, max_y], origin='lower', alpha=1, cmap=cmap,
                      norm=norm)
    if add_colorbar:
        fig.colorbar(patch, cax=cax, orientation='vertical', label=f'Z {units}', extend='both')
        # fig.colorbar(mappable=mpl.cm.ScalarMappable(cmap=cmap, norm=norm), label=f'Z {units}', extend='both', cax=cax)
    ax.set_title(title, fontsize=16)
    ax.set_xlabel(f'X [{units}]')
    ax.set_ylabel(f'Y [{units}]')
    ax.grid(visible=True, which='major', color='#666666', linestyle='-')

    # Show the minor grid lines with very faint and almost transparent grey lines
    ax.minorticks_on()
    ax.grid(visible=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
    return fig, ax, patch

def create_empty_fig(row_num: int = 1, col_num: int = 1):
    fig, axes = plt.subplots(row_num, col_num, figsize=(15, 10))
    [ax.set_xticks([]) for ax in axes.ravel()]
    [ax.set_yticks([]) for ax in axes.ravel()]
    [axes[i, 0].axis('off') for i in range(3, row_num)]

    return fig, axes


def plot_pc(vertices: np.ndarray):
    min_val_x = np.percentile(vertices[0, :], 5)
    max_val_x = np.percentile(vertices[0, :], 95)
    min_val_y = np.percentile(vertices[1, :], 5)
    max_val_y = np.percentile(vertices[1, :], 95)
    min_val_z = np.percentile(vertices[2, :], 5)
    max_val_z = np.percentile(vertices[2, :], 95)

    vertex_num = vertices.shape[1]
    vertices_x = np.clip(vertices[0, np.arange(1, vertex_num, 50)], min_val_x, max_val_x)
    vertices_y = np.clip(vertices[1, np.arange(1, vertex_num, 50)], min_val_y, max_val_y)
    vertices_z = np.clip(vertices[2, np.arange(1, vertex_num, 50)], min_val_z, max_val_z)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(vertices_x, vertices_y, vertices_z, marker='o')
    plt.show()


def show_image(image: np.ndarray, show_cv2: bool = False):
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    if show_cv2:
        viz_image = cv2.flip(image, 0)
        cv2.imshow("color", viz_image)
    else:
        plt.figure()
        plt.imshow(rgb_image, origin='lower')
        plt.show()


def show_depth_image(depth_image: np.ndarray, show_cv2: bool = False):
    min_val = np.percentile(depth_image, 5)
    max_val = np.percentile(depth_image, 95)
    pc_depth_view = np.clip(depth_image, min_val, max_val)

    if show_cv2:
        viz_image = cv2.flip(depth_image, 0)
        cv2.imshow("depth", viz_image)
    else:
        plt.figure()
        plt.imshow(pc_depth_view, cmap='YlOrBr', origin='lower')
        plt.show(block=False)


def show_depth_image_no_clip(depth_image: np.ndarray, show_cv2: bool = False):
    if show_cv2:
        viz_image = cv2.flip(depth_image, 0)
        cv2.imshow("depth", viz_image)
    else:
        plt.figure()
        plt.imshow(depth_image, cmap='YlOrBr', origin='lower')
        plt.show(block=False)


def show_depth_image_gray(depth_image, show_cv2: bool = False):
    min_val = np.percentile(depth_image, 5)
    max_val = np.percentile(depth_image, 95)
    pc_depth_view = np.clip(depth_image, min_val, max_val)
    if show_cv2:
        viz_image = cv2.flip(pc_depth_view, 0)
        cv2.imshow("depth", viz_image)
    else:
        plt.figure()
        plt.imshow(pc_depth_view, cmap='gray', origin='lower')
        plt.show(block=False)


def colorize_depth(data: np.array, cmap: str = 'hsv', norm: str = 'regular') -> np.array:
    """
    this function converts matrix from f[x, y] = color to RGB matrix where f[3, x, y] is the RGB of each pixel
    :param data: input matrix from which to extract colors
    :param cmap: which color pallete to use (string must be from allowed cmap options - check get_cmap for options)
    :param norm: normalization mechanism, if 'regular' than just normalizes by min and max, otherwise does similar
    to viaulizations (two step of matplotlib)
    :return:
    """
    colormap = plt.get_cmap(cmap)
    if norm == 'regular':
        data_norm = (np.max(data) - data) / (np.max(data) - np.min(data)) * 255

    elif norm == 'percentile':
        min_per = np.percentile(data, 3)
        max_per = np.percentile(data, 97)
        data_clip = np.clip(data, min_per, max_per)
        data_norm = (max_per - data_clip) / (max_per - min_per) * 255

    else:
        max_z = np.max(data)
        lognorm = mpl.colors.TwoSlopeNorm(vcenter=0, vmax=max_z)
        data_norm = lognorm(data) * 255

    data_rgb = (colormap(data_norm.astype(int))[:, :, :3] * 255).astype(np.uint8)
    return data_rgb


def colorize_depth_with_bounds(data: np.array, cmap: str = 'hsv', min_val=-10, max_val=250) -> np.array:
    """
    this function converts matrix from f[x, y] = color to RGB matrix where f[3, x, y] is the RGB of each pixel
    :param max_val:
    :param min_val:
    :param data: input matrix from which to extract colors
    :param cmap: which color pallete to use (string must be from allowed cmap options - check get_cmap for options)
    to viaulizations (two step of matplotlib)
    :return:
    """

    colormap = plt.get_cmap(cmap)
    data_clip = np.clip(data, min_val, max_val)
    data_norm = (max_val - data_clip) / (max_val - min_val) * 255

    data_rgb = (colormap(data_norm.astype(int))[:, :, :3] * 255).astype(np.uint8)
    return data_rgb


def get_cmap(n, map_str="hsv"):
    """
    this function creates colors for matplotlib based on the string (color pallate) and the number of colors wanted (n)
    :param n: number of colors
    :param map_str: which mapping to use
    :return: list of colors
    """
    cmap = plt.get_cmap(map_str)
    colors = [cmap(i) for i in np.linspace(0, 1, n)]
    return colors


def gen_arrow_head_marker(rot: float):
    """generate a marker to plot with matplotlib scatter, plot, ...

    https://matplotlib.org/stable/api/markers_api.html#module-matplotlib.markers

    rot=0: positive x direction
    Parameters
    ----------
    rot : float
        rotation in degree
        0 is positive x direction

    Returns
    -------
    arrow_head_marker : Path
        use this path for marker argument of plt.scatter
    scale : float
        multiply a argument of plt.scatter with this factor got get markers
        with the same size independent of their rotation.
        Paths are autoscaled to a box of size -1 <= x, y <= 1 by plt.scatter
    """
    arr = np.array([[.1, .3], [.1, -.3], [1, 0]])  # arrow shape
    angle = rot / 180 * np.pi
    rot_mat = np.array([
        [np.cos(angle), np.sin(angle)],
        [-np.sin(angle), np.cos(angle)]
    ])
    arr = np.matmul(arr, rot_mat)  # rotates the arrow

    # scale
    x0 = np.amin(arr[:, 0])
    x1 = np.amax(arr[:, 0])
    y0 = np.amin(arr[:, 1])
    y1 = np.amax(arr[:, 1])
    scale = np.amax(np.abs([x0, x1, y0, y1]))

    arrow_head_marker = mpl.path.Path(arr)
    return arrow_head_marker, scale


def plot_pose_vec(pose_vec: List[RealDozerPose], hold_on=False, fig=None, ax=None):

    xs = []
    ys = []
    yaws = []
    markersize = 20

    if fig is None:
        fig, ax = plt.subplots()
    for i, curr_pose_aruco_axis in enumerate(pose_vec):
        aruco_delta_position_body_axis = Position.from_array(np.array([17, 0, 0]))
        rotated_delta_position = rotate_points_from_body_to_inertial(body_position=aruco_delta_position_body_axis,
                                                                     rotation=curr_pose_aruco_axis.rotation).squeeze()

        curr_dozer_position_inertial_axis = translate_points(curr_pose_aruco_axis.position.reshape(3, 1),
                                                             rotated_delta_position)
        curr_pose = RealDozerPose.from_classes(position=curr_dozer_position_inertial_axis,
                                               rotation=curr_pose_aruco_axis.rotation,
                                               velocity=curr_pose_aruco_axis.velocity,
                                               timestamp=curr_pose_aruco_axis.timestamp,
                                               vehicle_id=curr_pose_aruco_axis.vehicle_id)

        xs.append(curr_pose.position.x)
        ys.append(curr_pose.position.y)
        yaws.append(curr_pose.rotation.yaw)
        marker, scale = gen_arrow_head_marker(curr_pose.rotation.yaw)
        ax.scatter(curr_pose.position.x, curr_pose.position.y, marker=marker, s=(markersize * scale) ** 2, color='k')
                   # ,alpha=1-(i+1)/len(pose_vec))

    if not hold_on:
        plt.show()


def fig2data(fig: plt.figure() = None):
    """
    :param fig:
    :return: np.ndarry of the image
    """
    fig.canvas.draw()
    # Now we can save it to a numpy array.
    data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    data = data.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    data_out = data.copy()[:, :, ::-1]  # BGR to RGB
    return data_out


def visualize_depth_with_pose(height_map: np.ndarray,
                              pose_vec: List[RealDozerPose],
                              lower_bound_w_h: np.ndarray,
                              upper_bound_w_h: np.ndarray):

    disp_hmap = colorize_depth_with_bounds(data=height_map,
                                           cmap='YlOrBr',
                                           min_val=lower_bound_w_h[2],
                                           max_val=upper_bound_w_h[2])

    fig, ax = plt.subplots()
    ax.imshow(disp_hmap, origin='lower')

    plot_pose_vec(pose_vec, fig=fig, ax=ax)
    depth_with_scatter = fig2data(fig)

    depth_with_scatter = cv2.cvtColor(depth_with_scatter, cv2.COLOR_RGB2BGR)
    depth_with_scatter = cv2.flip(depth_with_scatter, 0)
    cv2.imshow("Depth with Scatter", depth_with_scatter)
    cv2.waitkey(10)


def plot_current_real_state(curr_color_image: np.ndarray,
                            curr_depth_image: np.ndarray,
                            chosen_action: RealDozerAction,
                            curr_path: List[RealDozerPose],
                            state: State,
                            lower_bound_w_h: np.ndarray,
                            upper_bound_w_h: np.ndarray,
                            show_video: bool):

    if show_video:
        disp_depth_image = colorize_depth_with_bounds(data=curr_depth_image,
                                                      cmap='YlOrBr',
                                                      min_val=lower_bound_w_h[2],
                                                      max_val=upper_bound_w_h[2])

        disp_depth_image = cv2.cvtColor(disp_depth_image, cv2.COLOR_RGB2BGR)
        curr_color_image = cv2.flip(curr_color_image, 0)
        disp_depth_image = cv2.flip(disp_depth_image, 0)
        cv2.imshow("World Color", curr_color_image)
        cv2.imshow("World Depth", disp_depth_image)
        cv2.waitKey(10)

    fig, ax = plt.subplots()
    _, _, _ = plot_heightmap_with_contours(heightmap_raster=state.current_heightmap_raster,
                                           title="current obs",
                                           fig=fig, ax=ax,
                                           plot_contours=False,
                                           units= 'CM')

    plt.scatter(state.dozer_pose.position.x, state.dozer_pose.position.y, marker='*', s=50, c='k')

    fig_r, ax_r = plt.subplots()

    _, _, _ = plot_heightmap_with_contours(heightmap_raster=state.dozer_fov_relative_height_raster,
                                           title="fov height map",
                                           fig=fig_r, ax=ax_r,
                                           plot_contours=False,
                                           units='CM')

    if show_video:
        obs_hmap_data = fig2data(fig)
        cv2.imshow("Algo Depth", obs_hmap_data)
        relative_hmap_data = fig2data(fig_r)
        cv2.imshow("Algo Fov Depth", relative_hmap_data)
        cv2.waitKey(10)
        plt.close('all')

    if curr_path is not None:
        fig_p, ax_p = plt.subplots()

        depth_image_raster = Raster(raster_data=curr_depth_image,
                                    raster_res=state.current_heightmap_raster.raster_res,
                                    area_size=state.current_heightmap_raster.area_size)

        _, _, _ = plot_heightmap_with_contours(heightmap_raster=depth_image_raster,
                                               title="current state",
                                               fig=fig_p, ax=ax_p,
                                               plot_contours=False,
                                               units='CM')

        ax_p.scatter(chosen_action.position.x, chosen_action.position.y, color='g', s=30)

        plot_pose_vec(pose_vec=curr_path, hold_on=True, fig=fig_p, ax=ax_p)

        if show_video:
            obs_hmap_dozer_path_data = fig2data(fig_p)
            cv2.imshow("Dozer Path", obs_hmap_dozer_path_data)
            cv2.waitKey(10)
            plt.close('all')

    if not show_video:
        plt.show()
    else:
        plt.close('all')


