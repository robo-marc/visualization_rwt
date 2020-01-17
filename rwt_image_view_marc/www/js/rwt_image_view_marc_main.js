$(function () {

  ////////////////////////////////////////
  // variables

  var ros = new ROSLIB.Ros();

  var mjpegCanvas = null;
  var currentImageTopic = null;

  var currentAngle = 0;

  // unused
  var isInRecording = false;

  var isRefreshing = false;


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

  function resizeCanvas() {
    // resize canvas
    var $canvas = $('#canvas-area canvas');
    if ($canvas.size() === 0) {
      return;
    }

    var $canvasWrap = $('#canvas-area');
    var width = $canvasWrap.width();
    var height = 480 * width / 640.0;

    $canvas.prop('width', width);
    $canvas.prop('height', height);
    mjpegCanvas.width = width;
    mjpegCanvas.height = height;

    if (currentAngle % 180 === 0) { // when 0 or 180 degree
      $canvasWrap.css('height', height);
    } else { // when 90 or 270 degree
      $canvasWrap.css('height', width);
      rotateCanvas(0); // adjust position
    }
  }

  function splitTopicAndType(topic) {
    var index = -1;

    // Judgment by naming rules, but want to judge by topicType
    if ((index = topic.indexOf('/compressedDepth')) !== -1) {
      // ignore because unsupported format
    } else if ((index = topic.indexOf('/compressed')) !== -1) {
      return {
        topic: topic.substring(0, index),
        type: 'ros_compressed'
      };
    }

    // sensor_msgs/Image or unsupported format
    return {
      topic: topic,
      type: undefined
    };
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
    var divWidth = $('#canvas-area').width();
    currentImageTopic = topic;
    var topicAndType = splitTopicAndType(topic);
    mjpegCanvas = new MJPEGCANVAS.Viewer({
      divID: 'canvas-area',
      host: ros.url().hostname,
      topic: topicAndType.topic,
      type: topicAndType.type,
      width: divWidth,
      height: 480 * divWidth / 640.0
    });

    // display at current angle
    rotateCanvas(0);
  });

  $('#refresh-button').on('click', function (e) {
    e.preventDefault();
    if (ros.isConnected) {
      isRefreshing = true;
      ros.close();
    } else {
      ros.autoConnect();
    }
  });

  $('#save-button').on('click', function (e) {
    e.preventDefault();

    var filename = 'image.png';

    var canvas = $('#canvas-area canvas').get(0);
    if (canvas) {
      if (canvas.msToBlob) {
        var blob = canvas.msToBlob();
        window.navigator.msSaveBlob(blob, filename);
      } else {
        var downloadLink = document.getElementById('save-image-link');
        downloadLink.href = canvas.toDataURL('image/png');
        downloadLink.download = filename;
        downloadLink.click();
      }
    }
  });

  $('#rotate-left-button').on('click', function (e) {
    e.preventDefault();
    rotateCanvas(-90);
  });

  $('#rotate-right-button').on('click', function (e) {
    e.preventDefault();
    rotateCanvas(90);
  });

  $('#fit-button').on('click', function (e) {
    e.preventDefault();
    resizeCanvas();
  });

  // $(window).on('resize', function () {
  //   resizeCanvas();
  // });

  // obsolated
  // $('#record-button').on('click', function (e) {
  //   e.preventDefault();
  //   var $button = $(this);
  //   if (currentImageTopic) {
  //     if (!isInRecording) {
  //       var rosbagStartClient = new ROSLIB.Service({
  //         ros: ros,
  //         name: '/rosbag_record',
  //         serviceType: 'rwt_image_view/RosbagRecordRequest'
  //       });
  //       var topicRequest = new ROSLIB.ServiceRequest({
  //         topics: [currentImageTopic]
  //       });
  //       rosbagStartClient.callService(topicRequest,
  //         function (result) {
  //           isInRecording = true;
  //           $button.removeClass('btn-success')
  //             .addClass('btn-danger')
  //             .html('stop recording');
  //           // download
  //
  //         }
  //       );
  //     } else {
  //       isInRecording = false;
  //       var rosbagStopClient = new ROSLIB.Service({
  //         ros: ros,
  //         name: '/rosbag_record_stop',
  //         serviceType: 'std_srvs/Empty'
  //       });
  //       var emptyRequest = new ROSLIB.ServiceRequest({});
  //       rosbagStopClient.callService(emptyRequest,
  //         function (result) {
  //           // download here
  //           var html = '<div class="alert alert-info alert-dismissable" id="download-alert">'
  //             + '<button type="button" class="close" data-dismiss="alert" aria-hidden="true">&times;</button>'
  //             + '<a class="alert-link" href="/rwt_image_view/tmp.bag">download the bagfile from here via right-click</a>'
  //             + '</div>';
  //           //$button.html('<a href="/rwt_image_view/tmp.bag">download</a>');
  //           $('#topic-area').before(html);
  //           $button.removeClass('btn-danger')
  //             .addClass('btn-success')
  //             .html('record');
  //         }
  //       );
  //     }
  //   }
  // });


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

      var $select = $('#topic-select');
      $select.empty();
      $select.append(_.map(topics, function (topic) {
        return '<option value="' + topic + '">' + topic + '</option>';
      }).join('\n'));
    });
  });

  ros.on('close', function () {
    $('#topic-select').empty();
    if (isRefreshing) {
      isRefreshing = false;
      ros.autoConnect();
    }
  });


  ////////////////////////////////////////
  // start screen
  initScreen();
});
