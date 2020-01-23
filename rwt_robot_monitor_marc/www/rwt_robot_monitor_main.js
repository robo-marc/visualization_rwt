$(function () {
  var ros = new ROSLIB.Ros();
  ros.autoConnect();
  var robot_monitor = new ROSLIB.RWTRobotMonitor({
    ros: ros,
    last_time_id: '#number'
  });

  // dialog hide
  $('#status-dialog').addClass('dialog_hidden');

  // pause button click
  $('#pause-button').on('click', function (e) {
    e.preventDefault();
    robot_monitor.is_paused = true;
    $('#pause-button').hide();
    $('#start-button').show();
  });

  // start button click
  $('#start-button').on('click', function (e) {
    e.preventDefault();
    robot_monitor.is_paused = false;
    $('#pause-button').show();
    $('#start-button').hide();
  });

  // time list botton click 
  $('.time-list').on('click', 'li', function () {
    // monitor pause
    robot_monitor.is_paused = true;
    $('#pause-button').hide();
    $('#start-button').show();

    var num = $(this).attr('id').substr(3);
    var data = {};
    data = arrData[Math.abs(parseInt(num, 10) - (arrData.length - 1))];

    var msg = data[0];
    robot_monitor.showHistory(msg);
  });

  // dialog close
  $('#close').on('click', function () {
    $('#status-dialog').addClass('dialog_hidden');
    $('#table-type-4').empty();
  });

  // all device tree tggle
  $('.table').on('click', '.toggle-button', function () {
    toggleAllTable($(this).parent().parent());
  });

  // dialog_box
  $('.table').on('dblclick', '.data_1', function () {
    dialogDataName = $(this).attr('data-name');
    var the_directory = robot_monitor.history.root.findByName(dialogDataName);
    showDialog(the_directory);
  });

});
