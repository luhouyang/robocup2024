import pluginlib


def list_plugins():
    plugin_classes = pluginlib.list_plugins(
        "controller_interface::ControllerInterface")
    for plugin_class in plugin_classes:
        print(plugin_class)


if __name__ == "__main__":
    list_plugins()
