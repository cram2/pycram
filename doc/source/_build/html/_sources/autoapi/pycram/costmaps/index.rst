:py:mod:`pycram.costmaps`
=========================

.. py:module:: pycram.costmaps


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   pycram.costmaps.Rectangle
   pycram.costmaps.Costmap
   pycram.costmaps.OccupancyCostmap
   pycram.costmaps.VisibilityCostmap
   pycram.costmaps.GaussianCostmap
   pycram.costmaps.SemanticCostmap



Functions
~~~~~~~~~

.. autoapisummary::

   pycram.costmaps.plot_grid



Attributes
~~~~~~~~~~

.. autoapisummary::

   pycram.costmaps.cmap


.. py:class:: Rectangle


   A rectangle that is described by a lower and upper x and y value.

   .. py:attribute:: x_lower
      :type: float

      

   .. py:attribute:: x_upper
      :type: float

      

   .. py:attribute:: y_lower
      :type: float

      

   .. py:attribute:: y_upper
      :type: float

      

   .. py:method:: translate(x: float, y: float)

      Translate the rectangle by x and y


   .. py:method:: scale(x_factor: float, y_factor: float)

      Scale the rectangle by x_factor and y_factor



.. py:class:: Costmap(resolution: float, height: int, width: int, origin: pycram.datastructures.pose.Pose, map: numpy.ndarray, world: typing_extensions.Optional[pycram.datastructures.world.World] = None)


   The base class of all Costmaps which implements the visualization of costmaps
   in the World.

   The constructor of the base class of all Costmaps.

   :param resolution: The distance in metre in the real-world which is
    represented by a single entry in the costmap.
   :param height: The height of the costmap.
   :param width: The width of the costmap.
   :param origin: The origin of the costmap, in world coordinate frame. The origin of the costmap is located in the
    centre of the costmap.
   :param map: The costmap represents as a 2D numpy array.
   :param world: The World for which the costmap should be created.

   .. py:method:: visualize() -> None

      Visualizes a costmap in the BulletWorld, the visualisation works by
      subdividing the costmap in rectangles which are then visualized as pybullet
      visual shapes.


   .. py:method:: _chunks(lst: typing_extensions.List, n: int) -> typing_extensions.List

      Yield successive n-sized chunks from lst.

      :param lst: The list from which chunks should be yielded
      :param n: Size of the chunks
      :return: A list of size n from lst


   .. py:method:: close_visualization() -> None

      Removes the visualization from the World.


   .. py:method:: _find_consectuive_line(start: typing_extensions.Tuple[int, int], map: numpy.ndarray) -> int

      Finds the number of consecutive entries in the costmap which are greater
      than zero.

      :param start: The indices in the costmap from which the consecutive line should be found.
      :param map: The costmap in which the line should be found.
      :return: The length of the consecutive line of entries greater than zero.


   .. py:method:: _find_max_box_height(start: typing_extensions.Tuple[int, int], length: int, map: numpy.ndarray) -> int

      Finds the maximal height for a rectangle with a given width in a costmap.
      The method traverses one row at a time and checks if all entries for the
      given width are greater than zero. If an entry is less or equal than zero
      the height is returned.

      :param start: The indices in the costmap from which the method should start.
      :param length: The given width for the rectangle
      :param map: The costmap in which should be searched.
      :return: The height of the rectangle.


   .. py:method:: merge(other_cm: Costmap) -> Costmap

      Merges the values of two costmaps and returns a new costmap that has for
      every cell the merged values of both inputs. To merge two costmaps they
      need to fulfill 3 constrains:

      1. They need to have the same size
      2. They need to have the same x and y coordinates in the origin
      3. They need to have the same resolution

      If any of these constrains is not fulfilled a ValueError will be raised.

      :param other_cm: The other costmap with which this costmap should be merged.
      :return: A new costmap that contains the merged values


   .. py:method:: __add__(other: Costmap) -> Costmap

      Overloading of the "+" operator for merging of Costmaps. Furthermore, checks if 'other' is actual a Costmap and
      raises a ValueError if this is not the case. Please check :func:`~Costmap.merge` for further information of merging.

      :param other: Another Costmap
      :return: A new Costmap that contains the merged values from this Costmap and the other Costmap


   .. py:method:: partitioning_rectangles() -> typing_extensions.List[Rectangle]

      Partition the map attached to this costmap into rectangles. The rectangles are axis aligned, exhaustive and
      disjoint sets.

      :return: A list containing the partitioning rectangles



.. py:class:: OccupancyCostmap(distance_to_obstacle: float, from_ros: typing_extensions.Optional[bool] = False, size: typing_extensions.Optional[int] = 100, resolution: typing_extensions.Optional[float] = 0.02, origin: typing_extensions.Optional[pycram.datastructures.pose.Pose] = None, world: typing_extensions.Optional[pycram.datastructures.world.World] = None)


   Bases: :py:obj:`Costmap`

   The occupancy Costmap represents a map of the environment where obstacles or
   positions which are inaccessible for a robot have a value of -1.

   Constructor for the Occupancy costmap, the actual costmap is received
   from the ROS map_server and wrapped by this class. Meta-data about the
   costmap is also received from the map_server.

   :param distance_to_obstacle: The distance by which the obstacles should be
       inflated. Meaning that obstacles in the costmap are growing bigger by this
       distance.
   :param from_ros: This determines if the Occupancy map should be created
       from the map provided by the ROS map_server or from the World.
       If True then the map from the ROS map_server will be used otherwise
       the Occupancy map will be created from the World.
   :param size: The length of the side of the costmap. The costmap will be created
       as a square. This will only be used if from_ros is False.
   :param resolution: The resolution of this costmap. This determines how much
       meter one pixel in the costmap represents. This is only used if from_ros
       is False.
   :param origin: This determines the origin of the costmap. The origin will
       be in the middle of the costmap. This parameter is only used if from_ros
       is False.

   .. py:method:: _calculate_diff_origin(height: int, width: int) -> pycram.datastructures.pose.Pose

      Calculates the difference between the origin of the costmap
      as stated by the meta-data and the actual middle of the costmap which
      is used by PyCRAM to visualize the costmap. The origin as stated by the
      meta-data refers to the position of the global coordinate frame with
      the bottom left corner as reference.

      :param height: The height of the costmap
      :param width: The width of the costmap
      :return: The difference between the actual origin and center of the costmap


   .. py:method:: _get_map() -> numpy.ndarray
      :staticmethod:

      Receives the map array from the map_server converts it and into a numpy array.

      :return: The costmap as a numpy array.


   .. py:method:: _get_map_metadata() -> nav_msgs.msg.MapMetaData
      :staticmethod:

      Receives the meta-data about the costmap from the map_server and returns it.
      The meta-data contains things like, height, width, origin and resolution.

      :return: The meta-data for the costmap array.


   .. py:method:: _convert_map(map: numpy.ndarray) -> numpy.ndarray

      Converts the Occupancy Map received from ROS to be more consistent
      with how PyCRAM handles its costmap. Every possible cell for a robot to stand
      is set to one while anything else is set to zero. Additionally, this method
      also takes into account the distance_to_obstacle parameter and sets cell values
      that are too close to an obstacle to 0.

      :param map: The map that should be converted. Represented as 2d numpy array
      :return: The converted map. Represented as 2d numpy array.


   .. py:method:: create_sub_map(sub_origin: pycram.datastructures.pose.Pose, size: int) -> Costmap

      Creates a smaller map from the overall occupancy map, the new map is centered
      around the point specified by "sub_origin" and has the size "size". The
      resolution of the costmap stays the same for the sub costmap.

      :param sub_origin: The point in global coordinate frame, around which the sub costmap should be centered.
      :param size: The size the sub costmap should have.
      :return: The sub costmap, represented as 2d numpy array.


   .. py:method:: _create_from_world(size: int, resolution: float) -> numpy.ndarray

      Creates an Occupancy Costmap for the specified World.
      This map marks every position as valid that has no object above it. After
      creating the costmap the distance to obstacle parameter is applied.

      :param size: The size of this costmap. The size specifies the length of one side of the costmap. The costmap is created as a square.
      :param resolution: The resolution of this costmap. This determines how much meter a pixel in the costmap represents.


   .. py:method:: _chunks(lst: typing_extensions.List, n: int) -> typing_extensions.List

      Yield successive n-sized chunks from lst.

      :param lst: The list from which chunks should be yielded
      :param n: Size of the chunks
      :return: A list of size n from lst



.. py:class:: VisibilityCostmap(min_height: float, max_height: float, size: typing_extensions.Optional[int] = 100, resolution: typing_extensions.Optional[float] = 0.02, origin: typing_extensions.Optional[pycram.datastructures.pose.Pose] = None, world: typing_extensions.Optional[pycram.datastructures.world.World] = None)


   Bases: :py:obj:`Costmap`

   A costmap that represents the visibility of a specific point for every position around
   this point. For a detailed explanation on how the creation of the costmap works
   please look here: `PhD Thesis (page 173) <https://mediatum.ub.tum.de/doc/1239461/1239461.pdf>`_

   Visibility Costmaps show for every position around the origin pose if the origin can be seen from this pose.
   The costmap is able to deal with height differences of the camera while in a single position, for example, if
   the robot has a movable torso.

   :param min_height: This is the minimal height the camera can be. This parameter
       is mostly relevant if the vertical position of the camera can change.
   :param max_height: This is the maximal height the camera can be. This is
       mostly relevant if teh vertical position of the camera can change.
   :param size: The length of the side of the costmap, the costmap is created
       as a square.
   :param resolution: This parameter specifies how much meter a pixel in the
       costmap represents.
   :param origin: The pose in world coordinate frame around which the
       costmap should be created.
   :param world: The World for which the costmap should be created.

   .. py:method:: _create_images() -> typing_extensions.List[numpy.ndarray]

      Creates four depth images in every direction around the point
      for which the costmap should be created. The depth images are converted
      to metre, meaning that every entry in the depth images represents the
      distance to the next object in metre.

      :return: A list of four depth images, the images are represented as 2D arrays.


   .. py:method:: _depth_buffer_to_meter(buffer: numpy.ndarray) -> numpy.ndarray

      Converts the depth images generated by the World to represent
      each position in metre.

      :return: The depth image in metre


   .. py:method:: _generate_map()

      This method generates the resulting density map by using the algorithm explained
      in Lorenz Mösenlechners `PhD Thesis (page 178) <https://mediatum.ub.tum.de/doc/1239461/1239461.pdf>`_
      The resulting map is then saved to :py:attr:`self.map`



.. py:class:: GaussianCostmap(mean: int, sigma: float, resolution: typing_extensions.Optional[float] = 0.02, origin: typing_extensions.Optional[pycram.datastructures.pose.Pose] = None)


   Bases: :py:obj:`Costmap`

   Gaussian Costmaps are 2D gaussian distributions around the origin with the given mean and sigma

   This Costmap creates a 2D gaussian distribution around the origin with
   the specified size.

   :param mean: The mean input for the gaussian distribution, this also specifies
       the length of the side of the resulting costmap. The costmap is Created
       as a square.
   :param sigma: The sigma input for the gaussian distribution.
   :param resolution: The resolution of the costmap, this specifies how much
       meter a pixel represents.
   :param origin: The origin of the costmap around which it will be created.

   .. py:method:: _gaussian_window(mean: int, std: float) -> numpy.ndarray

      This method creates a window of values with a gaussian distribution of
      size "mean" and standart deviation "std".
      Code from `Scipy <https://github.com/scipy/scipy/blob/v0.14.0/scipy/signal/windows.py#L976>`_



.. py:class:: SemanticCostmap(object, urdf_link_name, size=100, resolution=0.02, world=None)


   Bases: :py:obj:`Costmap`

   Semantic Costmaps represent a 2D distribution over a link of an Object. An example of this would be a Costmap for a
   table surface.

   Creates a semantic costmap for the given parameter. The semantic costmap will be on top of the link of the given
   Object.

   :param object: The object of which the link is a part
   :param urdf_link_name: The link name, as stated in the URDF
   :param resolution: Resolution of the final costmap
   :param world: The World from which the costmap should be created

   .. py:method:: generate_map() -> None

      Generates the semantic costmap according to the provided parameters. To do this the axis aligned bounding box (AABB)
      for the link name will be used. Height and width of the final Costmap will be the x and y sizes of the AABB.


   .. py:method:: get_aabb_for_link() -> pycram.datastructures.dataclasses.AxisAlignedBoundingBox

      Returns the axis aligned bounding box (AABB) of the link provided when creating this costmap. To try and let the
      AABB as close to the actual object as possible, the Object will be rotated such that the link will be in the
      identity orientation.

      :return: Two points in world coordinate space, which span a rectangle



.. py:data:: cmap

   

.. py:function:: plot_grid(data: numpy.ndarray) -> None

   An auxiliary method only used for debugging, it will plot a 2D numpy array using MatplotLib.


