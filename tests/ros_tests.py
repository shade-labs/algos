import threading
import rclpy
import time
import os
import requests
from rclpy.node import Node
import subprocess

from posix import _exit

from std_msgs.msg import String

MAX_INIT_ATTEMPTS = 10

# shaderobotics/resnet
# Try to get the "resnet" component of any algorithm. This is used to identify that the topics have launched
ALGO_NICKNAME = os.getenv('ALGO').split('/')[1]

ALGO_DETAILS = requests.get('https://provisioning.shaderobotics.com/v1/algo', params={'algo': os.getenv('ALGO')}).json()


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('testing_node')
        # Thread kill flags
        self.subprocess = None

        # Wait 10 seconds for the algorithm to initialize
        attempts = 0
        while True:
            def check_if_algorithm_launched() -> bool:
                for topic in self.get_topic_names_and_types():
                    if ALGO_NICKNAME in topic[0]:
                        return True
                return False

            if check_if_algorithm_launched():
                break

            attempts += 1
            if attempts >= MAX_INIT_ATTEMPTS:
                raise TimeoutError(f"Could not find a topic matching {ALGO_NICKNAME} in {MAX_INIT_ATTEMPTS} seconds.")
            time.sleep(1)

        def determine_input_topic():
            # Determine the algorithm's input topic
            topics: dict = ALGO_DETAILS['topics']
            for topic, value in topics.items():
                if value['side'] == 'subscriber':
                    return topic

            raise LookupError("Could not determine algorithm input topic")

        def determine_output_topic():
            # Determine the algorithm's output topic
            publisher_list = []
            topics: dict = ALGO_DETAILS['topics']
            for topic, value in topics.items():
                if value['side'] == 'publisher':
                    publisher_list.append(topic)

            for topic in publisher_list:
                if 'detections' in topic:
                    return topic

            if len(publisher_list) > 0:
                return publisher_list[0]

            raise LookupError("Could not determine algorithm output topic")

        algorithm_input_topic = determine_input_topic()
        algorithm_output_topic = determine_output_topic()

        print(f"Algorithm input topic: {algorithm_input_topic} - algorithm output topic {algorithm_output_topic}")

        self.subscription = self.create_subscription(
            String,
            algorithm_output_topic,
            self.listener_callback,
            2
        )

        print(f"Subscribed to {algorithm_output_topic}")

        threading.Thread(target=self.start_publisher, args=(algorithm_input_topic, )).start()

        threading.Thread(target=self.kill_in_time, args=(MAX_INIT_ATTEMPTS, )).start()

    def kill_proc(self, exit_code: int):
        if self.subprocess is not None:
            self.subprocess.kill()
        _exit(exit_code)

    def start_publisher(self, input_topic):
        self.subprocess = subprocess.Popen(['python3', os.getcwd() + '/camera_simulator.py'],
                                           env={**dict(os.environ), **{"OUTPUT_TOPIC": input_topic}})

    def kill_in_time(self, seconds_to_live: int):
        time.sleep(seconds_to_live)
        print(f"Did not hear response in {seconds_to_live}")
        self.kill_proc(1)

    def listener_callback(self, msg):
        def check_all_topic_types():
            active_topics = self.get_topic_names_and_types()
            required_topics = ALGO_DETAILS['topics']
            for topic in required_topics:
                correct = False
                for active_topic in active_topics:
                    # When one of the active topics match a required topic
                    if topic == active_topic[0]:
                        # Now also check type
                        if active_topic[1][0] in required_topics[topic]['type']:
                            print(f'✅ {active_topic[0]} found and has correct type '
                                  f'{required_topics[active_topic[0]]["type"]}')
                            correct = True
                            break

                if not correct:
                    print(f"❌ Could not find {topic} of the correct name and type in {active_topics}")
                    self.kill_proc(1)

            return True

        check_all_topic_types()

        print("✅ Validated algorithm - exiting...")

        self.kill_proc(0)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
