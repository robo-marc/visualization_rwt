// RobotMonitor.js

/**
 * @fileOverview a file to define RWTRobotMonitor class.
 * @author F-ROSROBO
 */

// previous data
// var previousData = [];
var arrData = [];
var maxData = 30;

/**
 * a class to visualize diagnostics messages
 * @class RWTRobotMonitor
 * @param spec
 */
ROSLIB.RWTRobotMonitor = function (spec) {
  // spec, ros
  var diagnostics_agg_topic = spec.diagnostics_agg_topic || '/diagnostics_agg';
  var ros = spec.ros;
  this.last_diagnostics_update = null;
  this.last_time_id = spec.last_time_id;

  // defaults to 100
  this.maxData = spec.maxData || 30;
  this.clearData();

  this.history = new ROSLIB.DiagnosticsHistory(spec);
  this.diagnostics_agg_subscriber = new ROSLIB.Topic({
    ros: ros,
    name: diagnostics_agg_topic,
    messageType: 'diagnostic_msgs/DiagnosticArray'
  });
  var that = this;
  this.diagnostics_agg_subscriber.subscribe(function (msg) {
    // paused
    if (ROSLIB.RWTRobotMonitor.prototype.is_paused) {
      return;
    }
    that.diagnosticsCallback(msg);
    // // // TODO test
    // console.log('---- diagnosticsCallback msg ----');
    // console.log(msg);
    that.addData(msg);
  });

  // timer to update last_time_id
  setTimeout(function () {
    that.updateLastTimeString();
  }, 1000);
  $('#pause-button').show();
  $('#start-button').hide();
};

/**
 * callback function for /diagnostics_agg.
 * @param msg - message of /diagnostics_agg.
 */
ROSLIB.RWTRobotMonitor.prototype.diagnosticsCallback = function (msg) {
  this.last_diagnostics_update = ROSLIB.Time.now();
  var diagnostics_statuses
    = ROSLIB.DiagnosticsStatus.createFromArray(msg);

  // // TODO test
  // console.log('----- diagnostics_statuses ----');
  // console.log(diagnostics_statuses);

  var that = this;

  _.forEach(diagnostics_statuses, function (status) {
    that.history.registerStatus(status);

    // // TODO test
    // console.log('----- status ----');
    // console.log(status);

  });
  var history = false;
  this.updateView(history);
};

/**
 * callback function for /diagnostics_agg.
 * @param msg - message of /diagnostics_agg.
 */
ROSLIB.RWTRobotMonitor.prototype.showHistory = function (msg) {
  // this.last_diagnostics_update = ROSLIB.Time.now();
  var history = true;
  this.last_diagnostics_update = ROSLIB.Time.now();

  this.history = new ROSLIB.DiagnosticsHistory();

  var diagnostics_statuses
    = ROSLIB.DiagnosticsStatus.createFromArray(msg);

  var that = this;
  // // TODO test
  // console.log('----- show history ----');
  // console.log(diagnostics_statuses);

  // console.log('----- show that ----');
  // console.log(that);
  _.forEach(diagnostics_statuses, function (status) {
    that.history.registerStatus(status);
    // // TODO test
    // console.log('----- show status ----');
    // console.log(status);
  });
  this.updateView(history);
};

/**
 * callback function to update string to show the last message received
 */
ROSLIB.RWTRobotMonitor.prototype.updateLastTimeString = function () {
  var that = this;
  if (that.last_diagnostics_update) {
    var now = ROSLIB.Time.now();
    var diff = now.substract(that.last_diagnostics_update).toSec();
    $(that.last_time_id).html(Math.floor(diff));
  } else {
    $(that.last_time_id).html(-1);
  }
  setTimeout(function () {
    that.updateLastTimeString();
  }, 1000);
};

/**
 * update html view
 */
ROSLIB.RWTRobotMonitor.prototype.updateView = function (history) {
  var resultError = this.updateErrorList();
  var resultWarn = this.updateWarnList();
  this.updateAllList();
  if (!(history)) {
    this.updateHistory(resultError, resultWarn);
  }
  this.registerBrowserCallback();
};

ROSLIB.RWTRobotMonitor.prototype.updateList = function (list_id, level, icon) {
  $('#' + list_id + ' li').remove();
  var directories = this.history.root.getDirectories(level);
  directories.sort(function (a, b) {
    var apath = a.fullName();
    var bpath = b.fullName();
    if (apath > bpath) {
      return 1;
    } else if (bpath > apath) {
      return -1;
    } else {
      return 0;
    }
  });

  // TODO test
  console.log('---- directories ----');

  _.forEach(directories, function (dir) {
    var html_pre = '<li class="list-group-item" data-name="'
      + dir.fullName() + '"><span class="glyphicon ' + icon + '"></span>';
    var html_suf = '</li>';
    $('#' + list_id).append(html_pre
      + dir.fullName() + ':' + dir.status.message
      + html_suf);

    // TODO test
    console.log('---- dir.fullName ----');
    console.log(dir.fullName());
    console.log('---- dir.status.message ----');
    console.log(dir.status.message);
    console.log('---- dir ----');
    console.log(dir);

  });


  if (directories.length) {
    return true;
  } else {
    return false;
  }
};

/**
 * update history botton
 */
ROSLIB.RWTRobotMonitor.prototype.updateHistory = function (resultError, resultWarn) {

  var history = [];
  var btn = document.getElementById('btn0');
  var nextNum = 0;

  for (var dataIndex = this.maxData - 2; dataIndex >= 0; dataIndex--) {
    btn = document.getElementById('btn' + dataIndex.toString());
    nextNum = dataIndex + 1;

    switch (btn.style.background) {
      case 'red':
        btn = document.getElementById('btn' + nextNum.toString());
        btn.style.background = 'red';
        break;
      case 'yellow':
        btn = document.getElementById('btn' + nextNum.toString());
        btn.style.background = 'yellow';
        break;
      case 'green':
        btn = document.getElementById('btn' + nextNum.toString());
        btn.style.background = 'green';
        break;
    }
  }

  btn = document.getElementById('btn0');
  if (resultError) {
    btn.style.background = 'red';
  } else if (resultWarn) {
    btn.style.background = 'yellow';
  } else {
    btn.style.background = 'green';
  }
};

/**
 * update all list view
 */
ROSLIB.RWTRobotMonitor.prototype.updateAllList = function () {
  // check opened list first
  var open_ids = [];
  $('#all-list .in, #all-list .collapsing').each(function () {
    open_ids.push($(this).attr('id'));
  });
  $('#all-list li').remove();
  // return jquery object
  var rec = function (directory) {
    var $html = $('<li class="list-group-item inner" data-name="' + directory.fullName() + '">'
      + '<a data-toggle="collapse" data-parent="#all-list" href="#' + directory.uniqID() + '">'
      + directory.getCollapseIconHTML()
      + directory.getIconHTML() + directory.name + '</a>'
      + '</li>');
    if (directory.children.length === 0) {

      // TODO test
      console.log('---- $html ----');
      console.log(directory.fullName());
      console.log(directory.uniqID());
      console.log(directory.getCollapseIconHTML());
      console.log(directory.getIconHTML());
      console.log(directory.name);

      return $html;

    }
    else {
      var div_root = $('<ul class="list-group-item-content collapse no-transition" id="' + directory.uniqID() + '"></ul>');
      for (var j = 0; j < open_ids.length; j++) {
        if (open_ids[j].toString() === directory.uniqID().toString()) {
          //div_root.find('.collapse').addClass('in');
          div_root.addClass('in');
          break;
        }
      }
      //if (directory.uniqID().toString() === )
      for (var i = 0; i < directory.children.length; i++) {
        var the_child = directory.children[i];
        var the_result = rec(the_child);
        div_root.append(the_result);
      }
      $html.append(div_root);

      // TODO test
      console.log('---- div_root ----');
      console.log(div_root);

      return $html;
    }
  };

  for (var i = 0; i < this.history.root.children.length; i++) {
    var $html = rec(this.history.root.children[i]);

    // TODO test
    console.log('---- history.root.children ----');
    console.log(this.history.root.children[i]);

    $('#all-list').append($html);
  }
};

/**
 * update warn list view
 */
ROSLIB.RWTRobotMonitor.prototype.updateWarnList = function () {
  var resultWarn = this.updateList('warn-list', ROSLIB.DiagnosticsStatus.LEVEL.WARN, 'glyphicon-exclamation-sign');
  return resultWarn;
};

/**
 * update error list view
 */
ROSLIB.RWTRobotMonitor.prototype.updateErrorList = function () {
  var resultError = this.updateList('error-list', ROSLIB.DiagnosticsStatus.LEVEL.ERROR, 'glyphicon-minus-sign');
  return resultError;
};

/**
 * registering callbacks for clicking the view lists
 */
ROSLIB.RWTRobotMonitor.prototype.registerBrowserCallback = function () {
  var root = this.history.root;
  $('.list-group-item').dblclick(function () {
    if ($(this).find('.in').length !== 0) {
      return;                   // skip
    }
    var html = '<div class="modal fade" id="modal" tabindex="-1" role="dialog" aria-labelledby="myModalLabel" aria-hidden="true">'
      + '<div class="modal-dialog">'
      + '<div class="modal-content">'
      + '<div class="modal-header">'
      + '<button type="button" class="close dismiss-button" aria-hidden="true">&times;</button>'
      + '<h4 class="modal-title" id="myModalLabel">Modal title</h4>'
      + '</div>'
      + '<div class="modal-body">'
      + '</div>'
      + '<div class="modal-footer">'
      + '<button type="button" class="btn btn-primary dismiss-button">Close</button>'
      + '</div>'
      + '</div><!-- /.modal-content -->'
      + '</div><!-- /.modal-dialog -->'
      + ' </div><!-- /.modal -->';
    var the_directory = root.findByName($(this).attr('data-name'));
    var $html = $(html);
    var $first_body_html = $('<dl></dl>');
    var first_dict = {
      'Full name': the_directory.fullName(),
      'Component': the_directory.status.name,
      'Hardware ID': the_directory.status.hardware_id,
      'Level': the_directory.status.levelString(),
      'Message': the_directory.status.message
    };
    for (var first_key in first_dict) {
      $first_body_html.append('<dt>' + first_key + ':</dt>' + '<dd>' + first_dict[first_key] + '</dd>');
    }
    $html.find('.modal-body').append($first_body_html);
    var $second_body_html = $('<dl></dl>');
    for (var second_key in the_directory.status.values) {
      $second_body_html.append('<dt>' + second_key + ':</dt>' + '<dd>' + the_directory.status.values[second_key] + '</dd>');
    }
    $html.find('.modal-title').html(the_directory.fullName());

    $html.find('.modal-body').append($second_body_html);
    $html.find('.dismiss-button').click(function () {
      $html.on('hidden.bs.modal', function () {
        $('#modal').remove();
      });
      $html.modal('hide');
    });
    //$html.find('.modal-title').html()
    $('.container').append($html);
    $('#modal').modal();

  });
};

$('#pause-button').on('click', function () {
  ROSLIB.RWTRobotMonitor.prototype.is_paused = true;
  $('#pause-button').hide();
  $('#start-button').show();
});

$('#start-button').on('click', function () {
  ROSLIB.RWTRobotMonitor.prototype.is_paused = false;
  $('#pause-button').show();
  $('#start-button').hide();
});

// history click event
var historyClick = function (event) {
  // TODO test
  // console.log('----- ' + event.data.name + '  click----- ');

  // monitor pause
  ROSLIB.RWTRobotMonitor.prototype.is_paused = true;
  $('#pause-button').hide();
  $('#start-button').show();

  var num = event.data.name.substr(3);
  var data = {};
  data = arrData[Math.abs(parseInt(num, 10) - (arrData.length - 1))];
  // TODO test
  // console.log(data);

  var msg = data[0];
  // TODO test
  // console.log(msg);
  ROSLIB.RWTRobotMonitor.prototype.showHistory(msg);
};

// history botton click 
$('#btn0').on('click', { name: 'btn0' }, historyClick);
$('#btn1').on('click', { name: 'btn1' }, historyClick);
$('#btn2').on('click', { name: 'btn2' }, historyClick);
$('#btn3').on('click', { name: 'btn3' }, historyClick);
$('#btn4').on('click', { name: 'btn4' }, historyClick);
$('#btn5').on('click', { name: 'btn5' }, historyClick);
$('#btn6').on('click', { name: 'btn6' }, historyClick);
$('#btn7').on('click', { name: 'btn7' }, historyClick);
$('#btn8').on('click', { name: 'btn8' }, historyClick);
$('#btn9').on('click', { name: 'btn9' }, historyClick);
$('#btn10').on('click', { name: 'btn10' }, historyClick);
$('#btn11').on('click', { name: 'btn11' }, historyClick);
$('#btn12').on('click', { name: 'btn12' }, historyClick);
$('#btn13').on('click', { name: 'btn13' }, historyClick);
$('#btn14').on('click', { name: 'btn14' }, historyClick);
$('#btn15').on('click', { name: 'btn15' }, historyClick);
$('#btn16').on('click', { name: 'btn16' }, historyClick);
$('#btn17').on('click', { name: 'btn17' }, historyClick);
$('#btn18').on('click', { name: 'btn18' }, historyClick);
$('#btn19').on('click', { name: 'btn19' }, historyClick);
$('#btn20').on('click', { name: 'btn20' }, historyClick);
$('#btn21').on('click', { name: 'btn21' }, historyClick);
$('#btn22').on('click', { name: 'btn22' }, historyClick);
$('#btn23').on('click', { name: 'btn23' }, historyClick);
$('#btn24').on('click', { name: 'btn24' }, historyClick);
$('#btn25').on('click', { name: 'btn25' }, historyClick);
$('#btn27').on('click', { name: 'btn26' }, historyClick);
$('#btn27').on('click', { name: 'btn27' }, historyClick);
$('#btn28').on('click', { name: 'btn28' }, historyClick);
$('#btn29').on('click', { name: 'btn29' }, historyClick);

ROSLIB.RWTRobotMonitor.prototype.clearData = function () {
  // if (this.useTimestamp) {
  //   // Key: msgFieldPath without array index number
  //   // Value: dataItem array of series
  //   this.seriesMap = {};
  // }
  // else {
  this.data = new ROSLIB.RingBuffer({ bufferCount: this.maxData });
  // }
  // this.yMinValue = undefined;
  // this.yMaxValue = undefined;
  // this.needToAnimate = false;
  // if (this.spec) {
  //   $('#' + this.contentId).find('svg').remove();
  //   this.initializePlot(this.contentId, this.posisionId, this.legendId, this.spec);
  // }
};

ROSLIB.RWTRobotMonitor.prototype.addData = function (data) {
  // check the dimension
  var dataDimension = _.isArray(data) ? data.length : 0;
  if (dataDimension === 0) {
    data = [data];          // force to encapsulate into array
  }
  this.data.push(data);

  // TODO test
  // console.log('---- add data ----');
  // console.log(data);

  arrData = this.data.toArray();

  // TODO test
  // console.log('---- arrData ----');
  // console.log(arrData);

  // previousData.push(data);
  // console.log('---- previousData ----');
  // console.log(previousData);

  // var arrData = this.data.toArray();

};

/**
 * @fileOverview a file to define RingBuffer class
 * @author Ryohei Ueda
 */

/**
 * a class for ring buffer.
 * @class RingBuffer
 * @param spec
 * @property bufferCount
 * 
 */
ROSLIB.RingBuffer = function (spec) {
  this.bufferCount = (spec || {}).bufferCount || 30;
  this.clear();
};

ROSLIB.RingBuffer.prototype.push = function (data) {
  this.endIndex++;
  if (this.endIndex === this.bufferCount) {
    this.endIndex = 0;
  }
  if (this.count >= this.bufferCount) {
    this.buffer[this.endIndex] = data;
    // increment startIndex and endIndex
    this.startIndex++;
    if (this.startIndex === this.bufferCount) {
      this.startIndex = 0;
    }
  }
  else {                        // not filled yet
    this.buffer[this.endIndex] = data;
  }
  this.count++;

  // TODO test
  // console.log('---- startIndex ----');
  // console.log(this.startIndex);
  // console.log('---- endIndex ----');
  // console.log(this.endIndex);

  return data;
};

ROSLIB.RingBuffer.prototype.clear = function () {
  this.buffer = new Array(this.bufferCount);
  this.startIndex = 0;
  this.endIndex = -1;
  this.count = 0;
};

ROSLIB.RingBuffer.prototype.map = function (proc) {
  var ret = [];
  for (var i = this.startIndex; i < Math.min(this.count, this.bufferCount); i++) {
    ret.push(proc.call(this, this.buffer[i]));
  }
  if (this.count > this.bufferCount) {
    for (var j = 0; j < this.endIndex + 1; j++) {
      ret.push(proc.call(this, this.buffer[j]));
    }
  }

  // TODO test
  // console.log('---- map ret ----');
  // console.log(ret);
  return ret;
};

ROSLIB.RingBuffer.prototype.toArray = function () {
  return this.map(function (x) { return x; });
};

ROSLIB.RingBuffer.prototype.length = function () {
  return Math.min(this.bufferCount, this.count);
};
