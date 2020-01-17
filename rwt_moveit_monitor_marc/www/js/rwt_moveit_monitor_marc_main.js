$(function () {

  var robot_description = '/robot_description';
  var robot_description_semantic = '/robot_description_semantic';

  // subscribe topic
  var ros = new ROSLIB.Ros();
  // get parameters
  var paramGet1 = new ROSLIB.Param({
    ros: ros,
    name: robot_description,
  });
  var paramGet2 = new ROSLIB.Param({
    ros: ros,
    name: robot_description_semantic,
  });

  // initialize screen
  function initScreen() {
    // common
    ros.autoConnect();
    // get node
    setInterval(getNode, 5000);
    // get parameter
    setInterval(getParam, 5000);
  }

  // get node
  function getNode() {
    ros.getNodes(function (nodesData) {
      console.log('---- node Data ---');
      console.log(nodesData);
      if (nodesData.includes('/move_group')) {
        $('.status').text('Running');
      } else {
        $('.status').text('Not running');
      }
    });
  }

  // get parameter list
  function getParam() {
    paramGet1.get(function (value) {
      if (value === null) {
        $('#robot_description').text('No');
      } else {
        $('#robot_description').text('Yes');
      }
    });

    paramGet2.get(function (value) {
      if (value === null) {
        $('#robot_description_semantic').text('No');
      } else {
        $('#robot_description_semantic').text('Yes');
      }
    });
  }

  initScreen();

});