from ma_dozer.configs.config import Config
from ma_dozer.scripts.algo.algo_env import Env
from ma_dozer.scripts.algo.algo_messaging_thread import AlgoMessagingThread


def main():

    config = Config()
    print('algo starts...')
    algo_messaging_thread = AlgoMessagingThread(config=config)
    algo_messaging_thread.start()

    init_dozer_pose = algo_messaging_thread.read_pose(config.dozer.name)
    init_dumper_pose = algo_messaging_thread.read_pose(config.dumper.name)

    dozer_env = Env(config=config,
                    algo_messaging_thread=algo_messaging_thread,
                    name=config.dozer.name,
                    init_pose=init_dozer_pose)


    dozer_env.start()

    dumper_env = Env(config=config,
                     algo_messaging_thread=algo_messaging_thread,
                     name=config.dumper.name,
                     init_pose=init_dumper_pose)

    dumper_env.start()

    dozer_env.join()
    dumper_env.join()

    print('all agents finished their missions')
    return


if __name__ == '__main__':
    main()
