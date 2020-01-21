$(function () {

  ////////////////////////////////////////
  // variables

  var ros = new ROSLIB.Ros();

  var mjpegCanvas = null;
  var currentAngle = 0;
  var isRefreshing = false;


  ////////////////////////////////////////
  // functions

  // initialize screen
  function initScreen() {
    ros.autoConnect();
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

  ////////////////////////////////////////
  // ros events

  ros.on('connection', function () {
    var topicDefer = $.Deferred();
    var refineTypes = ['sensor_msgs/Image', 'sensor_msgs/CompressedImage'];
    var excludeItem = '/compressedDepth';
    ros.getTopics(function (result) {
      var topicList = [];
      for (var i = 0; i < result.types.length; i++) {
        if (refineTypes.indexOf(result.types[i]) >= 0 && result.topics[i].indexOf(excludeItem) === -1) {
          topicList.push(result.topics[i]);
        }
      }
      topicDefer.resolve(topicList);
    });

    topicDefer.promise().done(function () {
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
