from ma_dozer.configs.nodes_config import Topics, CameraNodeConfig, AlgoNode, DozerNode, DumperNode
from ma_dozer.configs.pydantic_config import BaseModel


class Config(BaseModel):

    topics: Topics = Topics()

    camera: CameraNodeConfig = CameraNodeConfig()
    algo: AlgoNode = AlgoNode()
    dozer: DozerNode = DozerNode()
    dumper: DumperNode = DumperNode()

    dozer_aruco_idx: int = 3 #TODO, move to camera config
    dumper_aruco_idx: int = 4 #TODO move to camera config
    use_estimated_aruco_pose = False
