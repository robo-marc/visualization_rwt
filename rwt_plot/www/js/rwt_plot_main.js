
$(function () {

  ////////////////////////////////////////
  // variables

  var plot = new ROSLIB.RWTPlot({
    max_data: 2,          // when using timestamp, it is regarded as seconds
    timestamp: true
  });

  var ros = new ROSLIB.Ros();

  var spec = {
    yaxis: {
      auto_scale: true,
      auto_scale_margin: 0.2,
      min: 0.1
    }
  };

  var sub = null;

  ////////////////////////////////////////
  // functions

  // initialize screen
  function initScreen() {

    plot.initializePlot($("#plot-area"), spec);

    // subscribe topic
    ros.install_config_button("config-button");

    $("#y-auto-check").click();
    $("#pause-button").show();
    $("#start-button").hide();

    printXAxisSec();
    printYAxisDomain();
  }

  function printXAxisSec() {
    var x_sec = plot.getXAxisSec();
    $("#x-sec").val(Math.round(x_sec));
  }

  function printYAxisDomain() {
    var y_domain = plot.getYAxisMinMax();
    var y_min = y_domain.min;
    var y_max = y_domain.max;
    $("#y-min").val(round10(y_min));
    $("#y-max").val(round10(y_max));
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

  ////////////////////////////////////////
  // screen events

  $("#subscribe-topic-button").on("click", function () {
    var topic_name = $("#topic-select").val();
    var accessor = $("#field-accessor").val().split("/");

    if (sub) {
      console.log("unsubscribe");
      sub.unsubscribe();
    }
    ros.getTopicType(topic_name, function (topic_type) {
      sub = new ROSLIB.Topic({
        ros: ros,
        name: topic_name,
        messageType: topic_type
      });
      plot.clearData();
      var i = 0;
      var prev = ROSLIB.Time.now();
      accessor = _.map(accessor, function (a) {
        if (a.match(/\[[\d]+\]/)) {
          var array_index = parseInt(a.match(/\[([\d]+)\]/)[1], 10);
          return [a.split("[")[0], array_index];
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

        //plot.addData(getValFromAccessor(msg, accessor));
        plot.addData(now,
          getValFromAccessor(msg, accessor));

        if ($("#y-auto-check").prop("checked")) {
          printYAxisDomain();
        }

        var diff = now.substract(prev);
        //if (diff.toSec() > 1.0) {
        //console.log('sec: ' + diff.toSec());
        //}
        prev = now;
      });
    });
    return false;
  });

  $("#topic-select").on("change", function () {
    var topic_name = $("#topic-select").val();
    ros.getTopicType(topic_name, function (topic_type) {
      ros.getMessageDetails(topic_type, function (details) {
        var decoded = ros.decodeTypeDefs(details);
        $("#message-detail").find("pre").html(JSON.stringify(decoded, null, "  ")); // pretty print
      });
    });
  });

  $("#y-auto-check").on("change", function () {
    var is_auto = $(this).prop('checked');

    $("#y-min").prop("disabled", is_auto);
    $("#y-max").prop("disabled", is_auto);

    if ($("#y-auto-check").prop('checked')) {
      printYAxisDomain();
    }

  });

  $("#apply-config-button").on("click", function () {
    if (!plot) {
      return;
    }

    //set X-axis
    var x_sec = toInt($("#x-sec").val());
    plot.setXAxisScale(x_sec);

    // set Y-axis
    var is_auto = $("#y-auto-check").prop('checked');
    if (is_auto) {
      plot.setYAxisScaleAuto();
    } else {
      var y_min = toFloat($("#y-min").val());
      var y_max = toFloat($("#y-max").val());

      plot.setYAxisMinMaxMnually(y_min, y_max);
    }

    printXAxisSec();
    printYAxisDomain();
  });

  $("#pause-button").on("click", function () {
    plot.pause();
    $("#pause-button").hide();
    $("#start-button").show();
  });

  $("#start-button").on("click", function () {
    plot.start();
    $("#pause-button").show();
    $("#start-button").hide();
  });

  $("#clear-button").on("click", function () {
    if (spec.yaxis.auto_scale !== $("#y-auto-check").prop('checked')) {
      $("#y-auto-check").click();
    }

    plot.clearData();
  });

  ////////////////////////////////////////
  // ros events

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

  ros.on("close", function () {
    $("#topic-select").empty();
  });

  ////////////////////////////////////////
  // start screen
  initScreen();
});
