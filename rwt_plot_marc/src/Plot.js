// Plot.js

/**
 * @fileOverview a file to define RWTPlot class.
 * @author F-ROSROBO
 */


// class
/**
 * a class for plotting
 * @class RWTPlot
 * @param spec
 */
ROSLIB.RWTPlot = function (spec) {
  var lineDrawingRefleshRateLimit = 60;

  this.maxData = spec.maxData || 100; // defaults to 100
  this.useTimestamp = spec.timestamp;

  this.plot = null;
  this.drawingp = false;
  this.isPaused = false;
  this.startTs = Date.now();
  this.lineDrawingThrottle = _.throttle(
    this.drawTimestampedData,
    Math.round(1000 / lineDrawingRefleshRateLimit)
  );
  this.clearData();
};


ROSLIB.RWTPlot.prototype.clearData = function () {
  if (this.useTimestamp) {
    // Key: msgFieldPath without array index number
    // Value: dataItem array of series
    this.seriesMap = {};
  }
  else {
    this.data = new ROSLIB.RingBuffer({ bufferCount: this.maxData });
  }
  this.yMinValue = undefined;
  this.yMaxValue = undefined;
  this.needToAnimate = false;
  if (this.spec) {
    $('#' + this.contentId).find('svg').remove();
    this.initializePlot(this.contentId, this.posisionId, this.legendId, this.spec);
  }
};

ROSLIB.RWTPlot.prototype.resizePlot = function () {
  if (!this.spec) {
    return;
  }

  var $content = $('#' + this.contentId);
  var width = $content.width();
  var height = $content.height();
  var margin = this.spec.margin || { top: 20, right: 20, bottom: 20, left: 40 };

  ///// resize svg /////
  $('svg.rwt-plot')
    .attr('width', width)
    .attr('height', height);

  ///// resize clip area /////
  this.svg.select('#clip > rect')
    .attr('width', width - margin.left - margin.right)
    .attr('height', height - margin.top - margin.bottom);

  ///// resize x-axis /////
  this.xScale.range([0, width - margin.left - margin.right]);
  this.x.scale(this.xScale);
  this.svg.select('.x.axis').call(this.x);

  ///// re-draw path /////
  // right edge of x-axis domain
  var xEndTime = this.xScale.domain()[1];
  var xEndRosTime = ROSLIB.Time.fromDate(xEndTime);

  this.lineDrawingThrottle(xEndRosTime);
};

ROSLIB.RWTPlot.prototype.pause = function () {
  this.isPaused = true;
  $('#' + this.contentId).addClass('scroll');
};

ROSLIB.RWTPlot.prototype.start = function () {
  this.isPaused = false;
  $('#' + this.contentId).removeClass('scroll');
};

ROSLIB.RWTPlot.prototype.initializePlot = function (contentId, posisionId, legendId, spec) {
  this.spec = spec || {};

  this.contentId = contentId;
  this.legendId = legendId;
  this.posisionId = posisionId;
  var $content = $('#' + contentId);
  var width = $content.width();
  var height = $content.height();
  var margin = spec.margin || { top: 20, right: 20, bottom: 20, left: 40 };
  var that = this;
  var color = spec.color || false;

  var xaxisSpec = spec.xaxis || {};
  var xDomainWidth = xaxisSpec.domainWidth || 2;
  this.xDomainWidth = xDomainWidth;

  var yaxisSpec = spec.yaxis || {};
  var yaxisMin = yaxisSpec.min || 0.0;
  var yaxisMax = yaxisSpec.max || 1.0;
  var yaxisTick = yaxisSpec.tick || false;
  this.yAutoscale = yaxisSpec.autoScale || false;
  this.yAutoscaleMargin = yaxisSpec.autoScaleMargin || 0.2;
  this.yaxisTick = yaxisTick;

  this.specifiedColor = color;
  this.margin = margin;
  this.scrollBeginPosX = undefined;
  this.scrollBeginMaxX = undefined;

  if (this.useTimestamp) {
    this.xScale = d3.time.scale().range([0, width - margin.left - margin.right]);
  }
  else {
    this.xScale = d3.scale.linear()
      .domain([0, this.xRange - 1])
      .range([0, width - margin.left - margin.right]);
  }
  if (this.yAutoscale) {
    this.yScale = d3.scale.linear()
      .domain(this.axisMinMaxWithMargin(yaxisMin, yaxisMax, this.yAutoscaleMargin))
      .range([height - margin.top - margin.bottom, 0]);
  }
  else {
    this.yScale = d3.scale.linear()
      .domain([yaxisMin, yaxisMax])
      .range([height - margin.top - margin.bottom, 0]);
  }

  this.svg = d3.select('#' + this.contentId).append('svg')
    .attr('class', 'rwt-plot')
    .attr('width', width)
    .attr('height', height)
    .append('g')
    .attr('transform', 'translate(' + margin.left + ',' + margin.top + ')');

  this.svg.append('defs').append('clipPath')
    .attr('id', 'clip')
    .append('rect')
    .attr('width', width - margin.left - margin.right)
    .attr('height', height - margin.top - margin.bottom);

  // draw x axis
  var st = this.startTs;
  var ticks = this.xDomainWidth + 1;
  this.x = d3.svg.axis()
    .scale(this.xScale)
    .orient('bottom')
    .ticks(ticks)
    .tickFormat(function (d, i) {
      var tmp = Math.ceil((ticks - 1) / 10.0);
      if (i % tmp === 0) {
        return Math.floor((d.getTime() - st) / 1000.0);
      } else {
        return '';
      }
    });

  this.svg.append('g')
    .attr('class', 'x axis')
    .attr('transform', 'translate(0,' + this.yScale(0) + ')')
    .call(this.x);
  this.y = d3.svg.axis().scale(this.yScale).orient('left');
  if (this.yaxisTick) {
    this.y = this.y.ticks(this.yaxisTick);
  }
  this.svg.append('g')
    .attr('class', 'y axis')
    .call(this.y);

  if (this.useTimestamp) {
    this.line = d3.svg.line()
      .x(function (d, i) { return that.xScale(d[0].toDate()); })
      .y(function (d, i) { return that.yScale(d[1]); });
    this.refreshXAxisDomainByRosTime(ROSLIB.Time.now());
  }
  else {
    this.line = d3.svg.line()
      .x(function (d, i) { return that.xScale(i); })
      .y(function (d, i) { return that.yScale(d); });
  }

  this.color = d3.scale.category10();
  this.colorIndex = 0;

  // Key: path id(msgFieldPath with array index number)
  // Value: d3-path object
  this.paths = {};

  // Key: path id(msgFieldPath with array index number)
  // Value: object(name, color)
  this.pathSettings = {};

  this.setMouseHandler();
  this.paintLegend();
};

ROSLIB.RWTPlot.prototype.setMouseHandler = function () {
  var that = this;

  var $positionLabel = $('#' + this.posisionId);

  d3.select('#' + this.contentId + ' > svg')
    .on('mousedown', function () {
      // start scrolling only from above svg
      that.beginScroll(this, that);
    })
    .on('touchstart', function () {
      // start scrolling only from above svg
      that.beginScroll(this, that);
    })
    .on('mousemove', function () {
      that.paintPosition(this, that, $positionLabel);
    })
    .on('click', function () {
      that.paintPosition(this, that, $positionLabel);
    })
    ;

  d3.select('body')
    .on('mouseup', function () {
      // Note: If text is selected, mouseup will not fire because it becomes a drag
      that.endScroll(this, that);
    })
    .on('dragend', function () {
      // End scrolling when dragging ends
      that.endScroll(this, that);
    })
    .on('touchend', function () {
      that.endScroll(this, that);
    })
    .on('touchcancel', function () {
      that.endScroll(this, that);
    })
    .on('mousemove', function () {
      that.scroll(this, that);
    })
    .on('touchmove', function () {
      that.scroll(this, that);
    })
    ;

};

ROSLIB.RWTPlot.prototype.paintPosition = function (e, plot, positionLabel) {
  // get mouse physical coordinates
  var m = d3.mouse(e);

  // convert from physical coordinates to coordinates on graph
  var x = plot.xScale.invert(m[0] - plot.margin.left);
  x = plot.round10((x.getTime() - plot.startTs) / 1000.0);

  var y = plot.round10(plot.yScale.invert(m[1] - plot.margin.top));

  positionLabel.text('x = ' + x + '　y = ' + y);
};

ROSLIB.RWTPlot.prototype.beginScroll = function (e, plot) {
  if (!plot.isPaused) {
    return;
  }

  // get mouse physical coordinates
  var m = d3.mouse(e);

  // convert from physical coordinates to coordinates on graph
  // date value of clicked position
  var xPointedTime = plot.xScale.invert(m[0] - plot.margin.left);

  // right edge of x-axis domain
  var xEndTime = this.xScale.domain()[1];

  plot.scrollBeginPosX = xPointedTime;
  plot.scrollBeginMaxX = xEndTime;
};

ROSLIB.RWTPlot.prototype.endScroll = function (e, plot) {
  plot.scrollBeginPosX = undefined;
  plot.scrollBeginMaxX = undefined;
};

ROSLIB.RWTPlot.prototype.scroll = function (e, plot) {
  if (!plot.isPaused) {
    return;
  }

  // data at the start of scrolling
  var xPointedTime = plot.scrollBeginPosX;
  var xEndTime = plot.scrollBeginMaxX;
  if (!xPointedTime || !xEndTime) {
    return;
  }

  // get mouse physical coordinates
  var m = d3.mouse(e);

  // convert from physical coordinates to coordinates on graph
  // date value of current position
  var xPointingTime = plot.xScale.invert(m[0] - plot.margin.left);

  var diff = xPointingTime.getTime() - xPointedTime.getTime();
  var newXEndTime = new Date(xEndTime.getTime() - diff);

  var newRosTime = ROSLIB.Time.fromDate(newXEndTime);

  // plot.drawTimestampedData(newRosTime);
  plot.lineDrawingThrottle(newRosTime);
};

ROSLIB.RWTPlot.prototype.round10 = function (value) {
  return Math.round(value * 10) / 10;
};

ROSLIB.RWTPlot.prototype.getNextColor = function () {
  var color = this.color(this.colorIndex);
  this.colorIndex++;
  return color;
};

ROSLIB.RWTPlot.prototype.getFieldPathId = function (msgFieldPath, index) {
  var s = msgFieldPath + '/' + String(index); // like '/array_sin_cos/data/0'
  s = s.replace(/\//g, '__'); // like '__array_sin_cos__data__0'
  return s;
};

ROSLIB.RWTPlot.prototype.getFieldPathName = function (msgFieldPath, index) {
  var s = msgFieldPath; // like '/array_sin_cos/data'
  if (index !== undefined) {
    s += '[' + String(index) + ']'; // like '/array_sin_cos/data[0]'
  }
  return s;
};

ROSLIB.RWTPlot.prototype.paintLegend = function () {
  var that = this;

  var html = '<ul>';
  _.each(Object.keys(this.paths), function (pathId, index) {
    var name = that.pathSettings[pathId].name;
    var color = that.pathSettings[pathId].color;
    html += '<li><span style="background: ' + color + ';"></span>' + name + '</li>';
  });
  html += '</ul>';

  var $legendArea = $('#' + this.legendId);
  $legendArea.empty();
  $legendArea.html(html);
};

ROSLIB.RWTPlot.prototype.allocatePath = function (id, color) {
  return this.svg.append('g')
    .attr('id', id)
    .attr('clip-path', 'url(#clip)')
    .append('path')
    .datum([])
    .attr('class', 'line')
    .style('stroke', function (d) {
      return color;
    })
    .attr('d', this.line);
};

ROSLIB.RWTPlot.prototype.allocatePathForArr = function (msgFieldPath, dataItem) {
  for (var i = 0; i < dataItem.length; i++) {
    var fieldPathId = this.getFieldPathId(msgFieldPath, i);
    if (!(fieldPathId in this.paths)) { // 'A in B' means 'B contains key A'
      var color = this.getNextColor();

      this.paths[fieldPathId] = this.allocatePath(fieldPathId, color);

      var indexForName = (dataItem.length === 1 ? undefined : i);
      var pathName = this.getFieldPathName(msgFieldPath, indexForName);

      this.pathSettings[fieldPathId] = {
        name: pathName,
        color: color,
      };
      this.paintLegend();
    }
  }
};

ROSLIB.RWTPlot.prototype.setMaxData = function (max) {
  if (!max) {
    return;
  }

  var shouldClear = false;
  if (!this.useTimestamp && this.maxData !== max) {
    shouldClear = true;
  }

  this.maxData = max;

  if (shouldClear) {
    this.clearData(); // re-build RingBuffer
  }
};

ROSLIB.RWTPlot.prototype.getMaxData = function () {
  return this.maxData;
};

ROSLIB.RWTPlot.prototype.axisMinMaxWithMargin = function (min, max, margin) {
  var yMiddle = (max + min) / 2.0;
  var yMin = (min - yMiddle) * (1.0 + margin) + yMiddle;
  var yMax = (max - yMiddle) * (1.0 + margin) + yMiddle;
  return [yMin, yMax];
};

ROSLIB.RWTPlot.prototype.checkYAxisMinMax = function (data) {
  var needToUpdate = false;
  for (var dataIndex = 0; dataIndex < data.length; dataIndex++) {
    var val = data[dataIndex];
    if (val < this.yMinValue || this.yMinValue === undefined) {
      this.yMinValue = val;
      needToUpdate = true;
    }
    if (val > this.yMaxValue || this.yMaxValue === undefined) {
      this.yMaxValue = val;
      needToUpdate = true;
    }
  }
  if (this.yAutoscale && needToUpdate) {
    this.yScale.domain(this.axisMinMaxWithMargin(this.yMinValue, this.yMaxValue,
      this.yAutoscaleMargin));
    var axis = d3.svg.axis().scale(this.yScale).orient('left');
    if (this.yaxisTick) {
      axis = axis.ticks(this.yaxisTick);
    }
    this.svg.select('.y.axis').call(axis);
  }
};

ROSLIB.RWTPlot.prototype.setYAxisMinMaxMnually = function (min, max) {
  var minToSet = min;
  var maxToSet = max;
  if (!minToSet) {
    minToSet = this.yMinValue;
  }
  if (!maxToSet) {
    maxToSet = this.yMaxValue;
  }
  this.yAutoscale = false;
  this.yScale.domain(this.axisMinMaxWithMargin(minToSet, maxToSet,
    this.yAutoscaleMargin));
  var axis = d3.svg.axis().scale(this.yScale).orient('left');
  if (this.yaxisTick) {
    axis = axis.ticks(this.yaxisTick);
  }
  this.svg.select('.y.axis').call(axis);
};

ROSLIB.RWTPlot.prototype.setYAxisScaleAuto = function () {
  this.yScale.domain(this.axisMinMaxWithMargin(this.yMinValue, this.yMaxValue,
    this.yAutoscaleMargin));
  var axis = d3.svg.axis().scale(this.yScale).orient('left');
  if (this.yaxisTick) {
    axis = axis.ticks(this.yaxisTick);
  }
  this.svg.select('.y.axis').call(axis);
};

ROSLIB.RWTPlot.prototype.setXAxisScale = function (sec) {
  if (!sec) {
    return;
  }

  if (this.useTimestamp) {
    if (sec > this.maxData) {
      sec = this.maxData;
    }
    this.xDomainWidth = sec;

    var newestStamp;
    _.each(this.seriesMap, function (dataList, fieldPath) {
      if (_.isArray(dataList) && dataList.length > 0) {
        var stamp = dataList[dataList.length - 1].stamp;
        if (newestStamp === undefined
          || stamp.toDate().getTime() > newestStamp.toDate().getTime()) {
          newestStamp = stamp;
        }
      }
    });
    if (newestStamp === undefined) {
      newestStamp = ROSLIB.Time.now();
    }
    this.refreshXAxisDomainByRosTime(newestStamp);
  } else {
    // cannot change domain
  }
};

ROSLIB.RWTPlot.prototype.refreshXAxisDomainByRosTime = function (xEndTime) {
  this.refreshXAxisDomainByDate(xEndTime.toDate());
};

ROSLIB.RWTPlot.prototype.refreshXAxisDomainByDate = function (xEndTime) {
  if (this.useTimestamp) {
    var st = this.startTs;
    var ticks = this.xDomainWidth + 1;

    if (!xEndTime) {
      xEndTime = new Date();
    }
    var xBeginTime = new Date(xEndTime.getTime() - this.xDomainWidth * 1000);

    this.xScale.domain([xBeginTime, xEndTime]);
    this.x.scale(this.xScale)
      .ticks(ticks)
      .tickFormat(function (d, i) {
        var tmp = Math.ceil((ticks - 1) / 10.0);
        if (i % tmp === 0) {
          return Math.floor((d.getTime() - st) / 1000.0); // sec
        } else {
          return '';
        }
      });
    this.svg.select('.x.axis')
      .call(this.x);
  } else {
    // cannot change domain
  }
};

ROSLIB.RWTPlot.prototype.getYAxisMinMax = function () {
  var domain = this.yScale.domain();
  var min = domain[0];
  var max = domain[1];
  var margin = this.yAutoscaleMargin;

  var middle = (min + max) / 2.0;
  min = (min - middle) / (1.0 + margin) + middle;
  max = (max - middle) / (1.0 + margin) + middle;

  return {
    min: min,
    max: max,
  };
};

ROSLIB.RWTPlot.prototype.getXAxisSec = function () {
  return this.xDomainWidth;
};

ROSLIB.RWTPlot.prototype.addRawData = function (data) {
  // check the dimension
  var dataDimension = _.isArray(data) ? data.length : 0;
  if (dataDimension === 0) {
    data = [data];          // force to encapsulate into array
  }
  this.checkYAxisMinMax(data);

  var now = ROSLIB.Time.now();
  this.data.push(data);
  var arrData = this.data.toArray();
  var decomposedArr = _.zip(arrData);
  if (this.paths.length < data.length) {
    for (var pathIndex = this.paths.length; pathIndex < data.length; pathIndex++) {
      this.paths.push(this.allocatePath(String(pathIndex), pathIndex % 7));
    }
  }
  if (this.isPaused) {
    return;
  }

  for (var i = 0; i < decomposedArr.length; i++) {
    var targetArr = decomposedArr[i];
    if (this.needToAnimate) {
      this.paths[i]
        .datum(targetArr)
        .attr('d', this.line)
        .attr('transform', null)
        .transition()
        .ease('linear')
        .attr('transform', 'translate(' + this.xScale(-1) + ',0)');
    }
    else {
      this.paths[i].datum(targetArr)
        .attr('d', this.line)
        .attr('transform', null)
        .transition();
    }
  }
  if (this.data.length() === this.maxData) {
    this.needToAnimate = true;
  }
};

ROSLIB.ROSTimeToSec = function (timea) {
  return timea.secs + timea.nsecs / 1000000000.0;
};

ROSLIB.ROSTimeDifference = function (timea, timeb) {
  return (timea.secs - timeb.secs) + (timea.nsecs - timeb.nsecs) / 1000000000.0;
};

ROSLIB.RWTPlot.prototype.chopTimestampedData = function (stamp) {
  var that = this;
  var isChopped = false;
  // check the oldest message
  _.each(this.seriesMap, function (dataArr, fieldPath) {
    if (_.isArray(dataArr) && dataArr.length > 0) {
      // chop here
      var chopNum = 0;
      for (var i = 0; i < dataArr.length; i++) {
        var diff = stamp.substract(dataArr[i].stamp).toSec();
        if (diff > that.maxData) {
          chopNum = chopNum + 1;
        }
        else {
          break;
        }
      }
      dataArr = dataArr.slice(chopNum);
      that.seriesMap[fieldPath] = dataArr;
      isChopped |= chopNum > 0;
    }
  });
  return isChopped;
};

ROSLIB.RWTPlot.prototype.addTimestampedData = function (msgFieldPath, stamp, dataItem) {
  var dataDimension = _.isArray(dataItem) ? dataItem.length : 0;
  if (dataDimension === 0) {
    dataItem = [dataItem];          // force to encapsulate into array
  }

  // isNumeric
  if (!($.isNumeric(dataItem[0]))) {
    return;
  }

  this.checkYAxisMinMax(dataItem);

  this.allocatePathForArr(msgFieldPath, dataItem);

  var dataArr = this.seriesMap[msgFieldPath];
  if (!dataArr) {
    dataArr = [];
  }

  this.chopTimestampedData(stamp);

  dataArr = this.seriesMap[msgFieldPath]; // get chopped data array
  if (!dataArr) {
    dataArr = [];
  }

  dataItem.stamp = stamp;
  dataArr.push(dataItem);
  this.seriesMap[msgFieldPath] = dataArr;

  if (dataArr.length < 1) {
    return;
  }
  if (this.isPaused) {
    return;
  }

  this.lineDrawingThrottle(stamp);

};

ROSLIB.RWTPlot.prototype.getBeginIndex = function (beginTime, searchTarget) {
  //binary search
  var left = 0;
  var right = searchTarget.length - 1;
  while (left < right) {
    var middle = Math.floor(left + (right - left) / 2);
    var stamp = searchTarget[middle].stamp.toNanoSec();
    if (stamp <= beginTime) {
      left = middle + 1;
    } else {
      right = middle;
    }
  }
  var beginIndex = left;
  return beginIndex;
};

ROSLIB.RWTPlot.prototype.drawTimestampedData = function (xEndTime) {
  this.refreshXAxisDomainByRosTime(xEndTime);

  if (!xEndTime) {
    xEndTime = ROSLIB.Time.now();
  }
  var xBeginTime = xEndTime.substract(ROSLIB.Time.fromSec(this.xDomainWidth));

  xEndTime = xEndTime.toNanoSec();
  xBeginTime = xBeginTime.toNanoSec() - 500000000; // begin before 0.5 sec from y-axis position
  var that = this;
  _.each(this.seriesMap, function (dataArr, msgFieldPath) {
    if (!dataArr) {
      dataArr = [];
    }

    var dataItemLength = 0;
    if (dataArr.length > 0) {
      dataItemLength = dataArr[0].length;
    }
    var beginIndex = that.getBeginIndex(xBeginTime, dataArr);
    for (var i = 0; i < dataItemLength; i++) { // x_i := i
      var fieldPathId = that.getFieldPathId(msgFieldPath, i);

      var plotData = [];
      var dataArrLength = dataArr.length;
      for (var j = beginIndex; j < dataArrLength; j++) {
        var stamp = dataArr[j].stamp.toNanoSec();
        if (stamp <= xEndTime) {
          var value = dataArr[j][i];
          var newData = [dataArr[j].stamp, value]; // [x1, y1] or [x1, z1]
          plotData.push(newData);
        } else {
          break;
        }
      }

      var needToAnimate = false;
      if (needToAnimate) {
        var translation = 0;
        that.paths[fieldPathId]
          .datum(plotData)
          .attr('d', that.line)
          .attr('transform', null)
          .transition()
          .ease('linear')
          .attr('transform', 'translate(' + (-translation) + ',0)');
      } else {
        if (fieldPathId in that.paths) {
          that.paths[fieldPathId]
            .datum(plotData)
            .attr('d', that.line)
            .attr('transform', null)
            .transition();
        }
      }
    }
  });
};

ROSLIB.RWTPlot.prototype.addData = function (msgFieldPath, data, data2) {
  if (this.useTimestamp) {
    this.addTimestampedData(msgFieldPath, data, data2);
  }
  else {
    this.addRawData(data);
  }
};

ROSLIB.RWTPlot.prototype.removeTimestampedSeries = function (msgFieldPath) {
  if (msgFieldPath in this.seriesMap) {
    var dataArr = this.seriesMap[msgFieldPath];
    if (dataArr && dataArr.length > 0) {
      var tmpData = dataArr[0];
      for (var i = 0; i < tmpData.length; i++) {
        var fieldPathId = this.getFieldPathId(msgFieldPath, i);
        this.svg.select('#' + fieldPathId)
          .remove();
        delete this.paths[fieldPathId];
        delete this.pathSettings[fieldPathId];
      }
    }
    delete this.seriesMap[msgFieldPath];
    this.paintLegend();
  }
};

ROSLIB.RWTPlot.prototype.removeSeries = function (msgFieldPath) {
  if (this.useTimestamp) {
    this.removeTimestampedSeries(msgFieldPath);
  }
  else {
    // not implemented
  }
};
