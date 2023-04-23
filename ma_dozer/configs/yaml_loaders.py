import yaml
from pydoc import locate
import numpy as np


def _extract_value_from_scalar_node(node: yaml.ScalarNode, mat_index_for_type: int = 0):
    var_type_str = node.tag.split(':')[-1]
    var_type = locate(var_type_str)

    return var_type(node.value)


class Array(yaml.YAMLObject):
    yaml_tag = '!Array'
    yaml_loader = yaml.SafeLoader

    @staticmethod
    def from_yaml(loader, node):
        list_array = [_extract_value_from_scalar_node(node) for node in node.value]
        return np.asarray(list_array).flatten().astype(type(list_array[0]))


class NDArray(yaml.YAMLObject):
    yaml_tag = '!NDArray'
    yaml_loader = yaml.SafeLoader

    @staticmethod
    def from_yaml(loader, node):
        mat_index_for_type = 2  # the first entry of the matrix decides its type
        var_type_str = node.value[mat_index_for_type].tag.split(':')[-1]
        var_type = locate(var_type_str)

        list_array = [var_type(node.value[i].value) for i in range(len(node.value))]
        row_num = int(list_array.pop(0))
        col_num = int(list_array.pop(0))
        return np.asarray(list_array).reshape(row_num, col_num).astype(var_type)


def ndarray_representer(dumper, array: np.ndarray):
    dump_list = array.flatten().tolist()

    if len(array.shape) == 1:
        return dumper.represent_sequence('!Array', dump_list, flow_style=False)
    elif len(array.shape) == 2:
        shape_list = list(array.shape)
        dump_list[:0] = shape_list
        return dumper.represent_sequence('!NDArray', dump_list, flow_style=False)

yaml.add_representer(np.ndarray, ndarray_representer)
