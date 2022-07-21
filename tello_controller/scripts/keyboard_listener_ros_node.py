from keyboard_listener_ros import KeyboardListenerRos


def main():
    keyboard_listener_ros = KeyboardListenerRos()
    keyboard_listener_ros.begin()
    keyboard_listener_ros.run()


if __name__ == "__main__":
    main()
