from dm_control import mjcf

class Hand():
    def __init__(self, xml_path, joint_names = None, actuator_names = None, name: str = None):
        self._mjcf_root = mjcf.from_path(xml_path)
        if name:
            self._mjcf_root.model = name
        # Find MJCF elements that will be exposed as attributes.
        # joints
        if joint_names is None:
            self._joints = self.mjcf_model.find_all('joint')
        else:
            self._joints = [self._mjcf_root.find('joint', name) for name in joint_names]
        # links
        self._bodies = self.mjcf_model.find_all('body')
        # motors
        if actuator_names is None:
            self._actuators = self.mjcf_model.find_all('actuator')
        else:
            self._actuators = [self._mjcf_root.find('actuator', name) for name in actuator_names]

    @property
    def joints(self):
        """List of joint elements belonging to the arm."""
        return self._joints

    @property
    def actuators(self):
        """List of actuator elements belonging to the arm."""
        return self._actuators

    @property
    def mjcf_model(self):
        """Returns the `mjcf.RootElement` object corresponding to this robot."""
        return self._mjcf_root