// RoslibExt.js

/**
 * @fileOverview a file to extend ROSLIB.
 *  these are should be merged into roslibjs.
 * @author Ryohei Ueda
 */

/**
 * Retrieves a type of ROS topic.
 *
 * @param callback - function with params:
 *   * type - String of the topic type
 */
ROSLIB.Ros.prototype.getTopicType = function (topic, callback) {
  var topicTypeClient = new ROSLIB.Service({
    ros: this,
    name: '/rosapi/topic_type',
    serviceType: 'rosapi/TopicType'
  });
  var request = new ROSLIB.ServiceRequest({
    topic: topic
  });
  topicTypeClient.callService(request, function (result) {
    callback(result.type);
  });
};

/**
 * Retrieves a detail of ROS message.
 *
 * @param callback - function with params:
 *   * details - Array of the message detail
 * @param message - String of a topic type
 */
ROSLIB.Ros.prototype.getMessageDetails = function (message, callback) {
  var messageDetailClient = new ROSLIB.Service({
    ros: this,
    name: '/rosapi/message_details',
    serviceType: 'rosapi/MessageDetails'
  });
  var request = new ROSLIB.ServiceRequest({
    type: message
  });
  messageDetailClient.callService(request, function (result) {
    callback(result.typedefs);
  });
};
