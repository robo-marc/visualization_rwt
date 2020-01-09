$(function () {

  ////////////////////////////////////////
  // variables

  var ros = new ROSLIB.Ros();

  var mjpegCanvas = null;
  var currentImageTopic = null;

  var currentAngle = 0;

  // unused
  var isInRecording = false;


  ////////////////////////////////////////
  // functions

  // initialize screen
  function initScreen() {
    ros.autoConnect();
  }

  function getTopicsForTypeAsync(type) {
    var defer = $.Deferred();

    ros.getTopicsForType(type,
      function (result) {
        // console.log('getTopicsForTypeAsync success');
        defer.resolve(result);
      },
      function (message) {
        console.log('getTopicsForTypeAsync failed: ' + message);
        defer.resolve([]);
      }
    );

    return defer.promise();
  }

  function rotateCanvas(degreeOffset) {
    currentAngle += degreeOffset;
    currentAngle = currentAngle % 360;
    if (currentAngle < 0) {
      currentAngle = 360 + currentAngle;
    }
    $('#angle').text(currentAngle);


    var $canvas = $('#canvas-area canvas');
    if ($canvas.size() === 0) {
      return;
    }

    var width = $canvas.prop('width');
    var height = $canvas.prop('height');
    var canvasTopOffset = 0;
    if (currentAngle % 180 === 0) { // when 0 or 180 degree
      canvasTopOffset = 0;
      $('#canvas-area').css('height', height);
    } else { // when 90 or 270 degree
      canvasTopOffset = (width - height) / 2;
      $('#canvas-area').css('height', width);
    }

    $canvas
      .css('transform-origin', 'center center')
      .css('transform', 'rotate(' + currentAngle + 'deg)')
      .css('position', 'relative')
      .css('top', canvasTopOffset)
      ;
  }


  ////////////////////////////////////////
  // screen events

  $('#view-button').on('click', function (e) {
    e.preventDefault();
    if (mjpegCanvas) {
      // remove the canvas here
      mjpegCanvas = null;
      $('#canvas-area canvas').remove();
    }
    var topic = $('#topic-select').val();
    // first of all, subscribe the topic and detect the width/height
    var div_width = $('#canvas-area').width();
    currentImageTopic = topic;
    mjpegCanvas = new MJPEGCANVAS.Viewer({
      divID: 'canvas-area',
      host: ros.url().hostname,
      topic: topic,
      width: div_width,
      height: 480 * div_width / 640.0
    });

    // display at current angle
    rotateCanvas(0);
  });

  $('#refresh-button').on('click', function (e) {
    e.preventDefault();
    // TODO:
  });

  $('#fit-button').on('click', function (e) {
    e.preventDefault();
    // TODO:
  });

  $('#save-button').on('click', function (e) {
    e.preventDefault();
    // TODO:
  });

  $('#rotate-left-button').on('click', function (e) {
    e.preventDefault();
    rotateCanvas(-90);
  });

  $('#rotate-right-button').on('click', function (e) {
    e.preventDefault();
    rotateCanvas(90);
  });

  // unused
  $('#record-button').on('click', function (e) {
    e.preventDefault();
    var $button = $(this);
    if (currentImageTopic) {
      if (!isInRecording) {
        var rosbagStartClient = new ROSLIB.Service({
          ros: ros,
          name: '/rosbag_record',
          serviceType: 'rwt_image_view/RosbagRecordRequest'
        });
        var topicRequest = new ROSLIB.ServiceRequest({
          topics: [currentImageTopic]
        });
        rosbagStartClient.callService(topicRequest,
          function (result) {
            isInRecording = true;
            $button.removeClass('btn-success')
              .addClass('btn-danger')
              .html('stop recording');
            // download

          }
        );
      } else {
        isInRecording = false;
        var rosbagStopClient = new ROSLIB.Service({
          ros: ros,
          name: '/rosbag_record_stop',
          serviceType: 'std_srvs/Empty'
        });
        var emptyRequest = new ROSLIB.ServiceRequest({});
        rosbagStopClient.callService(emptyRequest,
          function (result) {
            // download here
            var html = '<div class="alert alert-info alert-dismissable" id="download-alert">'
              + '<button type="button" class="close" data-dismiss="alert" aria-hidden="true">&times;</button>'
              + '<a class="alert-link" href="/rwt_image_view/tmp.bag">download the bagfile from here via right-click</a>'
              + '</div>';
            //$button.html('<a href="/rwt_image_view/tmp.bag">download</a>');
            $('#topic-area').before(html);
            $button.removeClass('btn-danger')
              .addClass('btn-success')
              .html('record');
          }
        );
      }
    }
  });


  ////////////////////////////////////////
  // ros events

  ros.on('connection', function () {
    var promises = [];
    promises.push(getTopicsForTypeAsync('sensor_msgs/Image'));
    promises.push(getTopicsForTypeAsync('sensor_msgs/CompressedImage'));

    $.when.apply(null, promises).done(function () {
      var topics = [];
      for (var i = 0; i < arguments.length; i++) {
        topics = topics.concat(arguments[i]);
      }
      topics.sort();
      $('#topic-select').append(_.map(topics, function (topic) {
        return '<option value="' + topic + '">' + topic + '</option>';
      }).join('\n'));
    });
  });

  ros.on('close', function () {
    $('#topic-select').empty();
  });


  ////////////////////////////////////////
  // start screen
  initScreen();
});
