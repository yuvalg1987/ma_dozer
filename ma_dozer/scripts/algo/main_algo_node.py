import cv2

from dozer_agents import dozer_agents_path
from dozer_agents.scripts.configs.inference_config import InferenceConfig
from dozer_agents.utils.inference.inference_utils import init_experiment, unravel_action_to_pixels

from ma_dozer.configs.config import Config
from ma_dozer.scripts.algo.algo_messaging_thread import AlgoMessagingThread

try:
    from matplotlib import pyplot as plt
except:
    import matplotlib as mpl
    mpl.use('Agg')
    from matplotlib import pyplot as plt


def main():

    is_done = False
    init_depth_image = None
    init_pose = None
    init_color_image = None

    config = Config()

    # camera_subscriber = CameraSubscriber(config=config)
    # camera_subscriber.start()

    algo_messaging_thread = AlgoMessagingThread(config=config)
    algo_messaging_thread.start()

    # dozer_path_subscriber = DozerPathSubscriber(config=config)
    # dozer_path_subscriber.start()

    curr_env_dir = dozer_agents_path / 'configs' / 'inference_prototype'
    base_inference_config_path = curr_env_dir / 'base_inference_config.yaml'
    base_scenario_config_path = curr_env_dir / 'base_scenario_config.yaml'

    inference_config = InferenceConfig.from_file(base_inference_config_path=base_inference_config_path,
                                                 base_scenario_config_path=base_scenario_config_path)

    while not is_done:
        print("planning snp next action")
        # get actions from policies (notice, each action is relevant to a different env based on the wanted wrapper!)
        snp_action = policies.expert(obs=expert_observation)
        learned_action = policies.learned(obs=learned_observation)

        if inference_config.base_config.show_videos or inference_config.base_config.show_debug:

            snp_action_unravel = unravel_action_to_pixels(action=snp_action,
                                                          fov_length_in_pixels=expert_env.dozer_fov_length_in_pixels,
                                                          fov_width_in_pixels=expert_env.dozer_fov_width_in_pixels)

            upsampled_learned_action = learned_env.get_action_from_specific_wrapper(action=learned_action,
                                                                                    wrapper_type=config.expert_config.wrapper_type)

            learned_action_unravel = unravel_action_to_pixels(action=upsampled_learned_action,
                                                              fov_length_in_pixels=expert_env.dozer_fov_length_in_pixels,
                                                              fov_width_in_pixels=expert_env.dozer_fov_width_in_pixels)

        # run action chosen
        if inference_config.base_config.run_expert:
            expert_observation, _, is_done, _ = expert_env.step(snp_action)
            learned_observation = learned_env.get_observation_from_specific_wrapper(obs=expert_observation,
                                                                                    wrapper_type=config.expert_config.wrapper_type)
        elif inference_config.base_config.run_agent:
            learned_observation, _, is_done, _ = learned_env.step(learned_action)
            # TODO : return expert_observation
        else:
            assert ValueError(f"no policy was chosen to run in real env!! must supply run_agent or run_policy in"
                              f"inference_config.base_config")


if __name__ == '__main__':
    main()
