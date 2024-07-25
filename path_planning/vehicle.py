class Vehicle:
    """
    Class to represent a vehicle to move in the map where the path should be found.
    """

    def __init__(self, width, height):
        """
        Initializes a vehicle.
        :param width: The width of the vehicle (in meters).
        :param height: The height of the vehicle (in meters).
        """
        super().__init__()
        self.width = width
        self.height = height
