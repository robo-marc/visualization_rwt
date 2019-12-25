
$(function () {

  // subscribe topic
  var ros = new ROSLIB.Ros();
  ros.install_config_button('config-button');

  ros.on('connection', function () {
    ros.getTopics(function (topic_info) {
      var topics = topic_info.topics;
      topics.sort();
      $('#topic-select').append(_.map(topics, function (topic) {
        return '<option value=' + topic + '>' + topic + '</option>';
      }).join('\n'));
      $('#topic-select').change();
    });
  });

  ros.on('close', function () {
    $('#topic-select').empty();
  });

  $('#execute-button').on('click', function () {
    var topic = $('#topic-select').val();

    //TODO: getTopictype 
    ros.getTopicType(topic, function (topicType) {

      console.log(topicType);
      var type = topicType;

      //TODO: getTopicInfo(hz,bw)
      ros.getTopicInfo(topic, type, function (value) {
        console.log(value);
      });
    });
  });

});
