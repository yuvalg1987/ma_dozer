from ma_dozer.configs.nodes_config import Topics, CameraNodeConfig, AlgoNode, DozerNode, DumperNode
from pydantic_config import BaseConfig


class Config(BaseConfig):

    topics: Topics = Topics()

    camera: CameraNodeConfig = CameraNodeConfig()
    algo: AlgoNode = AlgoNode()
    dozer: DozerNode = DozerNode()
    dumper: DumperNode = DumperNode()

    dozer_aruco_idx: int = 3
    use_estimated_aruco_pose = True
