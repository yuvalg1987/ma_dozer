from ma_dozer.configs.config import Array, NDArray
import yaml


def main():

    file_location = '/ma_dozer/configs/yaml_files/real_navigation_config.yaml'

    with open(file_location) as f:
        data = yaml.safe_load(f)


if __name__ == '__main__':

    main()