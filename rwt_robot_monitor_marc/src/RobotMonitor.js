// RobotMonitor.js

/**
 * @fileOverview a file to define RWTRobotMonitor class.
 * @author F-ROSROBO
 */

// previous data
// var previousData = [];
var arrData = [];
var maxData = 30;

// dialog hidden
$('#status-dialog').hide();

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
    // // paused
    // if (ROSLIB.RWTRobotMonitor.prototype.is_paused) {
    //   return;
    // }
    that.diagnosticsCallback(msg);
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
  // paused
  if (ROSLIB.RWTRobotMonitor.prototype.is_paused) {
    return;
  }
  var diagnostics_statuses
    = ROSLIB.DiagnosticsStatus.createFromArray(msg);
  var that = this;
  _.forEach(diagnostics_statuses, function (status) {
    that.history.registerStatus(status);
  });
  var history = false;
  this.updateView(history);
};

/**
 * callback function for /diagnostics_agg.
 * @param msg - message of /diagnostics_agg.
 */
ROSLIB.RWTRobotMonitor.prototype.showHistory = function (msg) {
  var history = true;
  this.last_diagnostics_update = ROSLIB.Time.now();
  this.history = new ROSLIB.DiagnosticsHistory();
  var diagnostics_statuses
    = ROSLIB.DiagnosticsStatus.createFromArray(msg);
  var that = this;
  _.forEach(diagnostics_statuses, function (status) {
    that.history.registerStatus(status);
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
  this.updateAllTable();
  if (!(history)) {
    this.updateTimeList(resultError, resultWarn);
  }
  this.registerBrowserCallback();
};

ROSLIB.RWTRobotMonitor.prototype.updateTable = function (list_id, tr_class, level) {

  //delete table
  $('#' + list_id + ' tr:gt(0)').remove();
  $('#' + list_id + ' tr:gt(0)').size();

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

  _.forEach(directories, function (dir) {
    var table = '<tr class="'
      + tr_class
      + '">'
      + '<td class="data_1" data-name="'
      + dir.fullName()
      + '">'
      + dir.fullName()
      + '</td>'
      + '<td class="data_2">'
      + dir.status.message
      + '</td>'
      + '</tr>';
    $('#' + list_id).append(table);
  });

  if (directories.length) {
    return true;
  } else {
    return false;
  }
};

/**
 * update all table view
 */
ROSLIB.RWTRobotMonitor.prototype.updateAllTable = function () {
  // check opened list first
  var open_ids = [];

  // TODO test
  console.log('----- #all-table ----');
  console.log($('#all-table'));
  // Tree collapsing check
  $('#all-table .collapse').each(function () {
    open_ids.push($(this).attr('id'));
  });
  // TODO test
  console.log('----- allTable open_ids ----');
  console.log(open_ids);

  //delete table
  $('#all-table tr:gt(0)').remove();
  $('#all-table tr:gt(0)').size();

  // return jquery object
  var rec = function (directory, indent, parentId) {

    // indent
    var leaf = ' leaf leaf' + indent;
    // toggle
    var toggle = '';

    // TODO test 
    if (directory.children.length === 0) {
      // toggle = ' collapse';
    } else {
      toggle = ' expand';
    }

    if (parentId === '') {
      // parentId = directory.uniqID();
    }

    var allTableTree = $('<tr '
      + 'id="'
      + directory.uniqID()
      + '" class="'
      + parentId
      + ' '
      + directory.getIconHTML2()
      + leaf
      + toggle
      + '">'
      + '<td class="data_1" data-name="'
      + directory.fullName()
      + '">'
      + '<span>'
      + directory.name
      + '</span>'
      + '</td>'
      + '<td class="data_2">'
      + directory.status.message
      + '</td>'
      + '</tr>');
    console.log('----- allTableTree_1 ----');
    console.log(allTableTree);

    if (directory.children.length === 0) {
      // last child
      return allTableTree;
    } else {
      for (var j = 0; j < open_ids.length; j++) {
        // if (open_ids[j].toString() === directory.uniqID().toString()) {
        // div_root.addClass('inner');
        // break;
        // }
      }
      indent++;
      for (var i = 0; i < directory.children.length; i++) {
        var the_child = directory.children[i];
        var the_result = rec(the_child, indent, directory.uniqID());
        allTableTree.after(the_result);
      }
      // TODO test
      console.log('----- allTableTree ----');
      console.log(allTableTree);
      // allTableTree.append(div_root);
      return allTableTree;
    }
  };

  for (var i = 0; i < this.history.root.children.length; i++) {
    var allTable = rec(this.history.root.children[i], 0, '');
    // TODO test
    console.log('----- allTable ----');
    console.log(allTable);
    $('#all-table').append(allTable);
  }

  for (var j = 0; j < open_ids.length; j++) {
    // if (open_ids[j].toString() === directory.uniqID().toString()) {
    // div_root.addClass('inner');
    // break;
    // }
    var parentId = '.' + open_ids[j];
    console.log('----- parentId ----');
    console.log(parentId);
    $(parentId).slideToggle(1);
  }

  // TODO test collapse all
  $('.robot-joy').slideToggle(1);
  $('.robot-dummy2').slideToggle(1);
  $('.robot-dummy').slideToggle(1);
  $('.robot').slideToggle(1);

};

// TODO delete
/**
 * update all list view
 */
ROSLIB.RWTRobotMonitor.prototype.updateAllList = function () {
  // check opened list first

  console.log('----- #all-list ----');
  console.log($('#all-list'));

  var open_ids = [];
  $('#all-list .in, #all-list .collapsing').each(function () {
    open_ids.push($(this).attr('id'));
  });
  console.log('----- open_ids ----');
  console.log(open_ids);
  $('#all-list li').remove();

  // return jquery object
  var rec = function (directory) {
    console.log('----- directory ----');
    console.log(directory);
    var $html = $('<li class="list-group-item inner" data-name="'
      + directory.fullName() + '">'
      + '<a data-toggle="collapse" data-parent="#all-list" href="#'
      + directory.uniqID() + '">'
      + directory.getCollapseIconHTML()
      + directory.getIconHTML() + directory.name + '</a>'
      + '</li>');
    if (directory.children.length === 0) {
      console.log('----- All list $html ----');
      console.log($html);
      return $html;
    }
    else {
      var div_root = $('<ul class="list-group-item-content collapse no-transition" id="'
        + directory.uniqID()
        + '"></ul>');
      for (var j = 0; j < open_ids.length; j++) {
        if (open_ids[j].toString() === directory.uniqID().toString()) {
          div_root.addClass('in');
          break;
        }
      }
      for (var i = 0; i < directory.children.length; i++) {
        var the_child = directory.children[i];
        var the_result = rec(the_child);
        div_root.append(the_result);
      }
      $html.append(div_root);
      console.log('----- div_root ----');
      console.log(div_root);
      return $html;
    }
  };
  for (var i = 0; i < this.history.root.children.length; i++) {
    var $html = rec(this.history.root.children[i]);
    $('#all-list').append($html);
    console.log('----- All list Root $html ----');
    console.log($html);
  }
};

/**
 * update warn list view
 */
ROSLIB.RWTRobotMonitor.prototype.updateWarnList = function () {
  var resultWarn = this.updateTable('warn-table', 'warn', ROSLIB.DiagnosticsStatus.LEVEL.WARN);
  return resultWarn;
};

/**
 * update error list view
 */
ROSLIB.RWTRobotMonitor.prototype.updateErrorList = function () {
  var resultError = this.updateTable('error-table', 'error', ROSLIB.DiagnosticsStatus.LEVEL.ERROR);

  return resultError;
};

/**
 * update time list
 */
ROSLIB.RWTRobotMonitor.prototype.updateTimeList = function (resultError, resultWarn) {

  var history = [];
  var btn = document.getElementById('btn0');
  var nextNum = 0;

  for (var dataIndex = this.maxData - 2; dataIndex >= 0; dataIndex--) {
    btn = document.getElementById('btn' + dataIndex.toString());
    nextNum = dataIndex + 1;

    switch (btn.className) {
      case 'time-item error':
        btn = document.getElementById('btn' + nextNum.toString());
        btn.className = 'time-item error';
        break;
      case 'time-item warn':
        btn = document.getElementById('btn' + nextNum.toString());
        btn.className = 'time-item warn';
        break;
      case 'time-item ok':
        btn = document.getElementById('btn' + nextNum.toString());
        btn.className = 'time-item ok';
        break;
    }
  }

  btn = document.getElementById('btn0');
  if (resultError) {
    btn.className = 'time-item error';
  } else if (resultWarn) {
    btn.className = 'time-item warn';
  } else {
    btn.className = 'time-item ok';
  }
};

/**
 * registering callbacks for clicking the view lists
 */
ROSLIB.RWTRobotMonitor.prototype.registerBrowserCallback = function () {
  var root = this.history.root;

  $('.data_1').on('dblclick', function () {
    var the_directory = root.findByName($(this).attr('data-name'));

    // dialog_box_1
    $('#dialog_header_title').text(the_directory.fullName());
    $('#dialog_full_name').text(the_directory.fullName());
    $('#dialog_component').text(the_directory.name);
    $('#dialog_hardware').text(the_directory.status.hardware_id);
    $('#dialog_level').text(the_directory.status.levelString());
    $('#dialog_message').text(the_directory.status.message);

    // dialog_box_
    for (var key in the_directory.status.values) {
      $('#table-type-4').append('<tr><th>' + key + '</th><td>' + the_directory.status.values[key] + '</td></tr>');
    }
    $('#status-dialog').show();
  });

  // dialog close
  $('#close').on('click', function () {
    $('#status-dialog').hide();
    $('#table-type-4').empty();
  });

  $('.expand').on('click', function () {

    var rec = function (parent) {

      var parentId = '.' + $(parent).attr('id');
      console.log('----- parentId ----');
      console.log(parentId);
      console.log('----- parent ----');
      console.log(parent);

      console.log('----- ($(parent) ----');
      console.log($(parent));

      $(parent).toggleClass('collapse');

      // TODO recursive processing
      $(parentId).slideToggle(1);
      // $(this).removeClass('expand');
      // $(this).addClass('collapse');
      // $(this).toggleClass('collapse');
    };

    var result = rec(this);
    console.log(result);
  });

  $('.collapse').on('click', function () {
    // TODO 
  });
};

ROSLIB.RWTRobotMonitor.prototype.clearData = function () {
  this.data = new ROSLIB.RingBuffer({ bufferCount: this.maxData });
};

ROSLIB.RWTRobotMonitor.prototype.addData = function (data) {
  // paused
  if (ROSLIB.RWTRobotMonitor.prototype.is_paused) {
    return;
  }
  // check the dimension
  var dataDimension = _.isArray(data) ? data.length : 0;
  if (dataDimension === 0) {
    data = [data];          // force to encapsulate into array
  }
  this.data.push(data);
  arrData = this.data.toArray();
};

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
  } else {
    this.buffer[this.endIndex] = data;
  }
  this.count++;
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
  return ret;
};

ROSLIB.RingBuffer.prototype.toArray = function () {
  return this.map(function (x) { return x; });
};

ROSLIB.RingBuffer.prototype.length = function () {
  return Math.min(this.bufferCount, this.count);
};

// time list click event
var timeListClick = function (event) {

  // monitor pause
  ROSLIB.RWTRobotMonitor.prototype.is_paused = true;
  $('#pause-button').hide();
  $('#start-button').show();

  var num = event.data.name.substr(3);
  var data = {};
  data = arrData[Math.abs(parseInt(num, 10) - (arrData.length - 1))];

  var msg = data[0];
  ROSLIB.RWTRobotMonitor.prototype.showHistory(msg);
};

$('#pause-button').on('click', function (e) {
  e.preventDefault();
  ROSLIB.RWTRobotMonitor.prototype.is_paused = true;
  $('#pause-button').hide();
  $('#start-button').show();
});

$('#start-button').on('click', function (e) {
  e.preventDefault();
  ROSLIB.RWTRobotMonitor.prototype.is_paused = false;
  $('#pause-button').show();
  $('#start-button').hide();
});

// time list botton click 
$('#btn0').on('click', { name: 'btn0' }, timeListClick);
$('#btn1').on('click', { name: 'btn1' }, timeListClick);
$('#btn2').on('click', { name: 'btn2' }, timeListClick);
$('#btn3').on('click', { name: 'btn3' }, timeListClick);
$('#btn4').on('click', { name: 'btn4' }, timeListClick);
$('#btn5').on('click', { name: 'btn5' }, timeListClick);
$('#btn6').on('click', { name: 'btn6' }, timeListClick);
$('#btn7').on('click', { name: 'btn7' }, timeListClick);
$('#btn8').on('click', { name: 'btn8' }, timeListClick);
$('#btn9').on('click', { name: 'btn9' }, timeListClick);
$('#btn10').on('click', { name: 'btn10' }, timeListClick);
$('#btn11').on('click', { name: 'btn11' }, timeListClick);
$('#btn12').on('click', { name: 'btn12' }, timeListClick);
$('#btn13').on('click', { name: 'btn13' }, timeListClick);
$('#btn14').on('click', { name: 'btn14' }, timeListClick);
$('#btn15').on('click', { name: 'btn15' }, timeListClick);
$('#btn16').on('click', { name: 'btn16' }, timeListClick);
$('#btn17').on('click', { name: 'btn17' }, timeListClick);
$('#btn18').on('click', { name: 'btn18' }, timeListClick);
$('#btn19').on('click', { name: 'btn19' }, timeListClick);
$('#btn20').on('click', { name: 'btn20' }, timeListClick);
$('#btn21').on('click', { name: 'btn21' }, timeListClick);
$('#btn22').on('click', { name: 'btn22' }, timeListClick);
$('#btn23').on('click', { name: 'btn23' }, timeListClick);
$('#btn24').on('click', { name: 'btn24' }, timeListClick);
$('#btn25').on('click', { name: 'btn25' }, timeListClick);
$('#btn27').on('click', { name: 'btn26' }, timeListClick);
$('#btn27').on('click', { name: 'btn27' }, timeListClick);
$('#btn28').on('click', { name: 'btn28' }, timeListClick);
$('#btn29').on('click', { name: 'btn29' }, timeListClick);
