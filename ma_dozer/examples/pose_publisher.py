from ma_dozer.configs.config import Config
from ma_dozer.utils.zmq.infrastructure import Publisher
import time


def main():
    config = Config()
    pos_publisher = Publisher(ip=config.camera.ip, port=config.camera.position_port)

    while True:
        print('choose topic:\n'
              '1- Dozer position\n'
              '2- Dumper position')

        topic_input = input()
        if topic_input == 'q':
            break
        elif topic_input == 't':
            print(time.time_ns())
            continue
        elif int(topic_input) == 1:
            topic = config.topics.topic_dozer_position
        elif int(topic_input) == 2:
            topic = config.topics.topic_dumper_position
        else:
            continue

        print('Enter position in the rigfht format:\n')
        message_data = input()
        if message_data == '0':
            message_data = f'0#0#0#0#0#0#0#0#0#0#{time.time_ns()}'
            print(f'message: {message_data}')

        pos_publisher.send(topic, message_data)
        print('message has been sent\n')

    print('Done!!!')


if __name__ == '__main__':
    main()