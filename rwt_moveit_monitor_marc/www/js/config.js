function createGetTopicList() {
    var topics = ['/pointcloud', '/pointcloud2', '/image', '/camera_info'];
    var types = ['sensor_msgs/PointCloud', 'sensor_msgs/PointCloud2', 'sensor_msgs/Image', 'sensor_msgs/CameraInfo'];

    var topicInfo = {
        topics: topics,
        types: types
    };

    var topicList = {
        topicInfo: topicInfo,
        type: 'rosgraph_msgs/Log',
    };
    return topicList;
}