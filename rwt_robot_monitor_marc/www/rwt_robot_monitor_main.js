$(function () {
  var ros = new ROSLIB.Ros();
  ros.autoConnect();
  var robot_monitor = new ROSLIB.RWTRobotMonitor({
    ros: ros,
    last_time_id: '#last-message-sec'
  });
});
