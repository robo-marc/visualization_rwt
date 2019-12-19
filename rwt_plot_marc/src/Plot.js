// Plot.js

/**
 * @fileOverview a file to define RWTPlot class.
 * @author Ryohei Ueda
 */


// class
/**
 * a class for plotting
 * @class RWTPlot
 * @param spec
 */
ROSLIB.RWTPlot = function (spec) {
  this.maxData = spec.maxData || 100; // defaults to 100
  this.useTimestamp = spec.timestamp;

  this.plot = null;
  this.drawingp = false;
  this.isPaused = false;
  this.startTs = Date.now();
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
  this.needToAnimate = false;
  if (this.spec) {
    $('#' + this.content.attr('id')).find('svg').remove();
    this.initializePlot(this.content, this.spec);
  }
};

ROSLIB.RWTPlot.prototype.pause = function () {
  this.isPaused = true;
};

ROSLIB.RWTPlot.prototype.start = function () {
  this.isPaused = false;
};

ROSLIB.RWTPlot.prototype.initializePlot = function ($content, spec) {
  this.spec = spec || {};
  this.content = $content;
  var width = $content.width();
  var height = $content.height();
  var margin = spec.margin || { top: 20, right: 20, bottom: 20, left: 40 };
  var that = this;
  var color = spec.color || false;
  var yaxisSpec = spec.yaxis || {};
  var yaxisMin = yaxisSpec.min || 0.0;
  var yaxisMax = yaxisSpec.max || 1.0;
  var yaxisTick = yaxisSpec.tick || false;

  this.yAutoscale = yaxisSpec.autoScale || false;
  this.yMinValue = yaxisMin;
  this.yMaxValue = yaxisMax;
  this.yAutoscaleMargin = yaxisSpec.autoScaleMargin || 0.2;
  this.yaxisTick = yaxisTick;
  this.specifiedColor = color;

  if (this.useTimestamp) {
    //this.xScale = d3.scale.linear().domain([0, this.maxData]).range([0, width - margin.left - margin.right]);
    this.xScale = d3.time.scale().range([0, width - margin.left - margin.right]);
    //this.xScale.ticks(d3.time.second, 1);
    //.domain([0, this.maxData])
  }
  else {
    this.xScale = d3.scale.linear()
      .domain([0, this.maxData - 1])
      .range([0, width - margin.left - margin.right]);
  }
  if (this.yAutoscale) {
    this.yScale = d3.scale.linear()
      .domain(this.axisMinMaxWithMargin(this.yMinValue, this.yMaxValue, this.yAutoscaleMargin))
      .range([height - margin.top - margin.bottom, 0]);
  }
  else {
    this.yScale = d3.scale.linear()
      .domain([yaxisMin, yaxisMax])
      .range([height - margin.top - margin.bottom, 0]);
  }

  this.svg = d3.select('#' + $content.attr('id')).append('svg')
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
  var ticks = this.maxData + 1;
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
    this.refreshXAxisDomain(ROSLIB.Time.now());
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

  this.drawLegend();
};

// TODO: deprecated
// ROSLIB.RWTPlot.prototype.setColor = function (color) {
//   this.specifiedColor = color;
//   var retFunc = function (_) { return color; };
//   for (var i = 0; i < this.paths.length; i++) {
//     this.paths[i].style('stroke', retFunc);
//   }
// };

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

ROSLIB.RWTPlot.prototype.drawLegend = function () {
  var that = this;
  var legendVals = [];
  var legendColors = {};
  $.each(Object.keys(this.paths), function (index, pathId) {
    var name = that.pathSettings[pathId].name;
    legendVals.push(name);
    legendColors[name] = that.pathSettings[pathId].color;
  });

  $('#legend-div').empty();

  var areaPadding = 16;
  var legend = d3.select('#legend-div')
    .append('svg')
    .style('width', '90%')
    // .style('border', '1px solid gray')
    .style('margin', '16px')
    .style('padding', String(areaPadding) + 'px')
    .selectAll('g')
    .data(legendVals)
    .enter()
    .append('g')
    .attr('class', 'legends')
    ;

  legend.append('rect') // 凡例の色付け四角
    .attr('x', 0)
    .attr('y', 15)
    .attr('width', 30)
    .attr('height', 2)
    .style('fill', function (d, i) {
      return legendColors[d];
    }) // 色付け
    ;
  legend.append('text')  // 凡例の文言
    .attr('x', 35)
    .attr('y', 20)
    .text(function (d, i) { return d; })
    .attr('class', 'textselected')
    .style('text-anchor', 'start')
    // .style('font-size', 12)
    ;

  var padding = 20;
  // var legendAreaWidth = legend[0].parentNode.clientWidth - (areaPadding * 2);
  legend.attr('transform', function (d, i) {
    {
      // 凡例をひとまず１行あたり３個固定で描画してみる
      var itemCountPerRow = 3;
      var rowIndex = Math.floor(i / itemCountPerRow); // 凡例の行のインデックス番号
      var begin = rowIndex * itemCountPerRow;
      var end = begin + itemCountPerRow - 1;
      // console.log('legend[' + i + ']: begin:' + begin + ', end:' + end);
      return 'translate('
        + (
          d3.sum(legendVals, function (e, j) {
            // console.log('sum[i, j]: [' + i + '][' + j + ']');
            // if (j < i) {
            if (begin <= j && j <= end && j < i) {
              // console.log('  width: [' + legend[0][j].getBBox().width + ']');
              return legend[0][j].getBBox().width;   // 各凡例の横幅サイズ取得
            } else {
              // console.log('  width: [' + 0 + ']');
              return 0;
            }
          }) + padding * (i % itemCountPerRow))
        + ','
        + (legend[0][0].getBBox().height + padding) * rowIndex // 凡例の各行の高さを計算
        + ')';
    }
  });
};

ROSLIB.RWTPlot.prototype.allocatePath = function (id, color) {
  var that = this;
  // this.color.domain(_.range(num)); // update the domain of the color
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
      this.drawLegend();
    }
  }
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
    if (val < this.yMinValue) {
      this.yMinValue = val;
      needToUpdate = true;
    }
    if (val > this.yMaxValue) {
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
    this.maxData = sec;

    var newestStamp;
    $.each(this.seriesMap, function (fieldPath, dataList) {
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
    this.refreshXAxisDomain(newestStamp);
  } else {
    // cannot change domain
  }
};

ROSLIB.RWTPlot.prototype.refreshXAxisDomain = function (xEndTime) {
  if (this.useTimestamp) {
    var st = this.startTs;
    var ticks = this.maxData + 1;

    if (!xEndTime) {
      xEndTime = ROSLIB.Time.now();
    }
    var xBeginTime = xEndTime.substract(ROSLIB.Time.fromSec(this.maxData));

    this.xScale.domain([xBeginTime.toDate(), xEndTime.toDate()]);
    this.x.scale(this.xScale)
      .ticks(this.maxData + 1)
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
  return this.maxData;
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
        //.duration(0)
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
  $.each(this.seriesMap, function (fieldPath, dataArr) {
    if (_.isArray(dataArr) && dataArr.length > 0) {
      // chop here
      var chopNum = 0;
      for (var i = 0; i < dataArr.length; i++) {
        var diff = stamp.substract(dataArr[i].stamp).toSec();
        if (diff > this.maxData) {
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

// unused
ROSLIB.RWTPlot.prototype.getDisplayData = function (stamp, dataArr) {
  if (dataArr.length > 0) {
    var chopNum = 0;
    for (var i = 0; i < dataArr.length; i++) {
      var diff = stamp.substract(dataArr[i].stamp).toSec();
      if (diff > this.maxData) {
        chopNum = chopNum + 1;
      } else {
        break;
      }
    }
    return { animate: (chopNum > 0), data: dataArr.slice(chopNum) };
  } else {
    return { animate: false, data: dataArr };
  }
};

ROSLIB.RWTPlot.prototype.addTimestampedData = function (msgFieldPath, stamp, dataItem) {
  var dataDimension = _.isArray(dataItem) ? dataItem.length : 0;
  if (dataDimension === 0) {
    dataItem = [dataItem];          // force to encapsulate into array
  }
  this.checkYAxisMinMax(dataItem);

  this.allocatePathForArr(msgFieldPath, dataItem);

  var dataArr = this.seriesMap[msgFieldPath];
  if (!dataArr) {
    dataArr = [];
  }

  var beforeChopOldestStamp = null;
  if (dataArr.length > 0) {
    beforeChopOldestStamp = dataArr[0].stamp;
  }

  var needToAnimate = this.chopTimestampedData(stamp);

  dataItem.stamp = stamp;
  dataArr.push(dataItem);
  this.seriesMap[msgFieldPath] = dataArr;

  if (dataArr.length < 1) {
    return;
  }
  if (this.isPaused) {
    return;
  }

  this.refreshXAxisDomain(stamp);

  // var dispDataTmp = this.getDisplayData(stamp, dataArr);
  // var dispData = dispDataTmp.data;
  // needToAnimate = dispDataTmp.animate;

  var afterChopOldestStamp = dataArr[0].stamp;
  for (var i = 0; i < dataItem.length; i++) { // x_i := i
    var fieldPathId = this.getFieldPathId(msgFieldPath, i);

    var plotData = [];
    for (var j = 0; j < dataArr.length; j++) {
      var value = dataArr[j][i];
      // scale x(i) here
      // afterChopOldestStamp = 0, stamp = this.maxData
      if (!stamp.equal(afterChopOldestStamp)) {
        var newData = [dataArr[j].stamp, value]; // [x1, y1] or [x1, z1]
        plotData.push(newData);
      }
    }

    if (needToAnimate) {
      var translation = afterChopOldestStamp.substract(beforeChopOldestStamp).toSec();

      this.paths[fieldPathId]
        .datum(plotData)
        .attr('d', this.line)
        .attr('transform', null)
        .transition()
        //.duration(0)
        .ease('linear')
        .attr('transform', 'translate(' + (-translation) + ',0)');
    } else {
      this.paths[fieldPathId].datum(plotData)
        .attr('d', this.line)
        .attr('transform', null)
        .transition();
    }
  }
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
    this.drawLegend();
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
