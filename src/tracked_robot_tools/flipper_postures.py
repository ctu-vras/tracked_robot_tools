import copy

import rospy


class SymmetricalFlipperPose(object):
    def __init__(self, front, rear):
        self.front = front
        self.rear = rear

    def __str__(self):
        return "front: " + str(self.front) + ", rear: " + str(self.rear)

    def __repr__(self):
        return "[" + str(self.front) + ", " + str(self.rear) + "]"


class FlipperPostures(object):
    def __init__(self, init_values=None, postures_class=None):
        self._postures = list()
        self._postures_class = postures_class

        if init_values is None:
            init_values = rospy.get_param("flipper_postures")

        for item in init_values:
            if len(item) != 2 or (not isinstance(item[0], float) and not isinstance(item[0], int)) \
                    or (not isinstance(item[1], float) and not isinstance(item[1], int)):
                rospy.logwarn("Ignoring invalid flipper posture, it has to be an array of two doubles")
                continue
            self._postures.append(SymmetricalFlipperPose(float(item[0]), float(item[1])))

        rospy.loginfo("The following flipper postures are defined:")
        rospy.loginfo("\n".join(str(self).split("\n")[1:]))

    def get(self, index):
        return self._postures[index]

    def __len__(self):
        return len(self._postures)

    def all(self):
        return copy.copy(self._postures)

    def __str__(self):
        s = "FlipperPostures:\n"
        for i in range(len(self)):
            s += " - " + str(i) + ": " + str(self.get(i))
            if self._postures_class is not None:
                name = None
                for k in self._postures_class.__dict__:
                    if self._postures_class.__dict__[k] == i:
                        name = k
                        break
                if name is not None:
                    s += " (" + name + ")"
            s += "\n"
        return s

    def __repr__(self):
        return repr(self._postures)


