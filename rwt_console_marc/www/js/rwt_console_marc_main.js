
$(function() {

  // subscribe topic
  var ros = new ROSLIB.Ros();
  ros.install_config_button("config-button");

  ros.on("connection", function() {
    ros.getTopics(function(topic_info) {
      var topics = topic_info.topics;
      topics.sort();
      $("#topic-select").append(_.map(topics, function(topic) {
        return '<option value="' + topic + '">' + topic + "</option>";
      }).join("\n"));
      $("#topic-select").change();
    });
  });

  ros.on("close", function() {
    $("#topic-select").empty();
  });
});