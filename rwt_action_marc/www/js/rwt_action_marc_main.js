
$(function () {

  // subscribe topic
  var ros = new ROSLIB.Ros();
  ros.install_config_button("config-button");

  ros.on("connection", function () {
    ros.getTopics(function (topic_info) {
      var topics = topic_info.topics;
      topics.sort();
      $("#topic-select").append(_.map(topics, function (topic) {
        return '<option value="' + topic + '">' + topic + "</option>";
      }).join("\n"));
      $("#topic-select").change();
    });
  });

  ros.getActionList(function (result) {
    var objList = [];
    _.each(result.message, function (msg, index) {
      var replace = msg.split('\'').join('\"');
      var obj = JSON.parse(replace);
      objList.push(obj);
    }, function (message) {
      console.log(message);
    });
    console.log(objList);
  });

  ros.on("close", function () {
    $("#topic-select").empty();
  });
});
