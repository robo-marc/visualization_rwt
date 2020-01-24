
$(function () {

  ////////////////////////////////////////
  // variables

  var plotSpec = {
    maxData: 120,          // when using timestamp, it is regarded as seconds
    timestamp: true
  };

  var initPlotSpec = {
    xaxis: {
      domainWidth: 2,          // when using timestamp, it is regarded as seconds
    },
    yaxis: {
      autoScale: true,
      autoScaleMargin: 0.2,
      min: 0.1
    }
  };

  // Key: msgFieldPath(topicName + '/' + accessor)
  // Value: ROSLIB.Topic object
  var subscribingMap = {};

  var plot = new ROSLIB.RWTPlot(plotSpec);

  var ros = new ROSLIB.Ros();

  ////////////////////////////////////////
  // functions

  // initialize screen
  function initScreen() {

    ros.autoConnect();

    plot.initializePlot('plot-area', 'position-area', 'legend-area', initPlotSpec);

    $('#y-auto-check').click();
    $('#pause-button').show();
    $('#start-button').hide();
    $('#open_sub_button').click();

    printPlotSpec();
    printXAxisSec();
    printYAxisDomain();
  }

  function printPlotSpec() {
    var max = plot.getMaxData();
    $('#max-data').val(max);
  }

  function printXAxisSec() {
    var xSec = plot.getXAxisSec();
    $('#x-range').val(Math.round(xSec));
  }

  function printYAxisDomain() {
    var yDomain = plot.getYAxisMinMax();
    var yMin = yDomain.min;
    var yMax = yDomain.max;
    $('#y-min').val(round10(yMin));
    $('#y-max').val(round10(yMax));
  }

  function toInt(value) {
    if ($.isNumeric(value)) {
      return Math.round(parseFloat(value));
    } else {
      return undefined;
    }
  }

  function toFloat(value) {
    if ($.isNumeric(value)) {
      return parseFloat(value);
    } else {
      return undefined;
    }
  }

  function round10(value) {
    return Math.round(value * 10) / 10;
  }

  function getValFromAccessor(msg, accessor) {
    if (accessor.length === 0) {
      return msg;
    }
    else {
      if (_.isArray(accessor[0])) {
        return getValFromAccessor(msg[accessor[0][0]][accessor[0][1]], accessor.slice(1));
      }
      else {
        return getValFromAccessor(msg[accessor[0]], accessor.slice(1));
      }
    }
  }

  function getFieldList(typeInfo) {
    var typeList = [];

    _.each(typeInfo, function (fieldType, fieldName) {
      if (_.isArray(fieldType)) {
        typeList.push(fieldName);

      } else if (typeof fieldType === 'object') {
        var children = getFieldList(fieldType);
        _.each(children, function (child, index) {
          var s = fieldName + '/' + child;
          typeList.push(s);
        });

      } else {
        if (isNumericTypeName(fieldType)) {
          typeList.push(fieldName);
        }
      }
    });

    return typeList;
  }

  function isNumericTypeName(name) {
    return (/^(int|uint|float|ufloat)/.test(name));
  }

  function containsKey(map, key) {
    return key in map;
  }

  ////////////////////////////////////////
  // screen events

  $('#remove-topic-button').on('click', function () {
    var msgFieldPath = $('#subscribed-select').val();
    if (subscribingMap[msgFieldPath]) {
      subscribingMap[msgFieldPath].unsubscribe();
      delete subscribingMap[msgFieldPath];
      plot.removeSeries(msgFieldPath);
      $('#subscribed-select option[value="' + msgFieldPath + '"]').remove();
    }
  });

  $('#add-topic-button').on('click', function () {
    var topicName = $('#topic-select').val();
    var typeName = $('#type-select').val();
    var accessor = typeName.split('/');

    var msgFieldPath = topicName + '/' + typeName;
    if (containsKey(subscribingMap, msgFieldPath)) {
      console.log('topic already subscribed: %s', msgFieldPath);
      return;
    }
    subscribingMap[msgFieldPath] = undefined;
    $('<option>', {
      value: msgFieldPath,
      text: msgFieldPath,
    }).appendTo('#subscribed-select');

    ros.getTopicType(topicName, function (topicType) {
      var sub = new ROSLIB.Topic({
        ros: ros,
        name: topicName,
        messageType: topicType
      });
      subscribingMap[msgFieldPath] = sub;
      accessor = _.map(accessor, function (a) {
        if (a.match(/\[[\d]+\]/)) {
          var arrayIndex = parseInt(a.match(/\[([\d]+)\]/)[1], 10);
          return [a.split('[')[0], arrayIndex];
        }
        else {
          return a;
        }
      });
      sub.subscribe(function (msg) {
        var now = null;
        if (msg.header && msg.header.stamp) {
          now = ROSLIB.Time.fromROSMsg(msg.header.stamp);
        }
        else {
          now = ROSLIB.Time.now();
        }

        plot.addData(msgFieldPath, now,
          getValFromAccessor(msg, accessor));

        if ($('#y-auto-check').prop('checked')) {
          printYAxisDomain();
        }
      });
    });
    return false;
  });

  $('#topic-select').on('change', function () {
    var topicName = $('#topic-select').val();
    ros.getTopicType(topicName, function (topicType) {
      ros.getMessageDetails(topicType, function (details) {
        var decoded = ros.decodeTypeDefs(details);

        var typeList = getFieldList(decoded);
        $('#type-select').val('');
        $('#type-select-data').empty();
        _.each(typeList, function (value, index) {
          $('<option>', {
            value: value,
            text: value,
          }).appendTo('#type-select-data');
        });
      });
    });
  });

  $('#y-auto-check').on('change', function () {
    var isAuto = $(this).prop('checked');

    $('#y-min').prop('disabled', isAuto);
    $('#y-max').prop('disabled', isAuto);

    if ($('#y-auto-check').prop('checked')) {
      printYAxisDomain();
    }
  });

  $('#apply-config-button').on('click', function () {
    if (!plot) {
      return;
    }

    //set plot spec
    var max = toInt($('#max-data').val());
    plot.setMaxData(max);

    //set X-axis
    var xSec = toInt($('#x-range').val());
    plot.setXAxisScale(xSec);

    // set Y-axis
    var isAuto = $('#y-auto-check').prop('checked');
    if (isAuto) {
      plot.setYAxisScaleAuto();
    } else {
      var yMin = toFloat($('#y-min').val());
      var yMax = toFloat($('#y-max').val());

      plot.setYAxisMinMaxMnually(yMin, yMax);
    }

    printPlotSpec();
    printXAxisSec();
    printYAxisDomain();
  });

  $('#pause-button').on('click', function (e) {
    e.preventDefault();
    plot.pause();
    $('#pause-button').hide();
    $('#start-button').show();
  });

  $('#start-button').on('click', function (e) {
    e.preventDefault();
    plot.start();
    $('#pause-button').show();
    $('#start-button').hide();
  });

  $('#clear-button').on('click', function (e) {
    e.preventDefault();
    if (initPlotSpec.yaxis.autoScale !== $('#y-auto-check').prop('checked')) {
      $('#y-auto-check').click();
    }

    plot.clearData();

    printPlotSpec();
    printXAxisSec();
    printYAxisDomain();
  });

  $('#open_sub_button').on('click', function (e) {
    e.preventDefault();
    $('#contents_sub').slideDown('normal');
    $('#close_sub_button').show();
    $('#open_sub_button').hide();
  });

  $('#close_sub_button').on('click', function (e) {
    e.preventDefault();
    $('#contents_sub').slideUp('normal');
    $('#close_sub_button').hide();
    $('#open_sub_button').show();
  });

  $(window).on('resize', function () {
    plot.resizePlot();
  });

  ////////////////////////////////////////
  // ros events

  ros.on('connection', function () {
    ros.getTopics(function (topicInfo) {
      var topics = topicInfo.topics;
      topics.sort();
      $('#topic-select').append(_.map(topics, function (topic) {
        return '<option value="' + topic + '">' + topic + '</option>';
      }).join('\n'));
      $('#topic-select').change();
    });
  });

  ros.on('close', function () {
    $('#topic-select').empty();
  });

  ////////////////////////////////////////
  // start screen
  initScreen();
});
