// RobotMonitor.js

/**
 * @fileOverview a file to define RWTRobotMonitor class.
 * @author F-ROSROBO
 */

// previous data
var arrData = [];
// time list
var maxData = 30;
// dialog 
var dialogDataName = '';
// all device init
var allTableInit = true;

// dialog hidden
$('#status-dialog').addClass('dialog_hidden');

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

/**
 * update table view  error/warn device
 */
ROSLIB.RWTRobotMonitor.prototype.updateTable = function (list_id, tr_class, level) {

  //delete table
  $('#' + list_id + ' tr:gt(0)').remove();
  $('#' + list_id + ' tr:gt(0)').size();

  var directories = this.history.root.getDirectories(level);

  // sort
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
 * update table view all device
 */
ROSLIB.RWTRobotMonitor.prototype.updateAllTable = function () {
  // check collapsed list from last time
  var collapseIdList = [];

  $('#all-table .collapse').each(function () {
    collapseIdList.push($(this).attr('id'));
  });

  //delete table
  $('#all-table tr:gt(0)').remove();
  $('#all-table tr:gt(0)').size();

  // return jquery object
  var rec = function (directory, indent, parentId, toggleParent, displayParent) {
    // indent
    var leaf = ' leaf leaf' + indent;
    // toggle (expand/collapse)
    var toggle = '';
    // display
    var display = '';

    // toggle check 
    if (directory.children.length !== 0) {
      if (allTableInit) {
        toggle = ' collapse';
      } else {
        for (var k = 0; k < collapseIdList.length; k++) {
          if (collapseIdList[k].toString() === directory.uniqID().toString()) {
            toggle = ' collapse';
            break;
          }
        }
      }
      if (toggle === '') {
        toggle = ' expand';
      }
    }

    // display check
    if (parentId && toggleParent === ' collapse') {
      display = ' collapsed';
    }
    if (parentId && displayParent === ' collapsed') {
      display = ' collapsed';
    }

    var allTableTree = $(''
      + '<tr '
      + 'id="'
      + directory.uniqID()
      + '" class="'
      + parentId
      + ' '
      + directory.getIconHTML2()
      + leaf
      + toggle
      + display
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

    // last child
    if (directory.children.length === 0) {
      return allTableTree;
    } else {
      indent++;
      for (var i = 0; i < directory.children.length; i++) {
        var the_child = directory.children[i];
        var the_result = rec(the_child, indent, directory.uniqID(), toggle, display);
        allTableTree.after(the_result);
      }
      return allTableTree;
    }
  };

  for (var i = 0; i < this.history.root.children.length; i++) {
    var allTable = rec(this.history.root.children[i], 0, '');
    $('#all-table').append(allTable);
  }
  allTableInit = false;
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

  // dialog_box
  $('.data_1').on('dblclick', function () {
    dialogDataName = $(this).attr('data-name');
    var the_directory = root.findByName(dialogDataName);
    dialogDisplay(the_directory);
  });

  // dialog close
  $('#close').on('click', function () {
    $('#status-dialog').addClass('dialog_hidden');
    $('#table-type-4').empty();
  });

  // dialog update
  if (!($('#status-dialog').hasClass('dialog_hidden'))) {
    $('#table-type-4').empty();
    var the_directory = root.findByName(dialogDataName);
    dialogDisplay(the_directory);
  }

  // dialog
  function dialogDisplay(the_directory) {
    // dialog_box_1
    $('#dialog_header_title').text(the_directory.fullName());
    $('#dialog_full_name').text(the_directory.fullName());
    $('#dialog_component').text(the_directory.name);
    $('#dialog_hardware').text(the_directory.status.hardware_id);
    $('#dialog_level').text(the_directory.status.levelString());
    $('#dialog_message').text(the_directory.status.message);

    // dialog_box_2
    for (var key in the_directory.status.values) {
      $('#table-type-4').append('<tr><th>'
        + key + '</th><td>'
        + the_directory.status.values[key]
        + '</td></tr>');
    }
    $('#status-dialog').removeClass('dialog_hidden');
  }

  // all device tree tggle
  $('tr.expand').on('click', function () {
    toggleAllTable(this);
  });

  // all device tree tggle
  $('tr.collapse').on('click', function () {
    toggleAllTable(this);
  });

  // toggle (expand/collapse)
  function toggleAllTable(parent) {

    var rec = function (parentId, collapse) {
      var parentClass = '.' + parentId;

      if (collapse) {
        $(parentClass).addClass('collapsed');
      } else {
        $(parentClass).toggleClass('collapsed');
      }

      var child = document.getElementsByClassName(parentId);
      for (var i = 0; i < child.length; i++) {
        var the_child = child[i];
        if ($(the_child).hasClass('collapse')) {
          collapse = 'collapse';
        } else {
          collapse = '';
        }
        rec($(the_child).attr('id'), collapse);
      }
    };

    $(parent).toggleClass('collapse');
    $(parent).toggleClass('expand');
    var collapse = '';
    if ($(parent).hasClass('collapse')) {
      collapse = 'collapse';
    }
    var result = rec($(parent).attr('id'), collapse);
  }

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
$('.time-list').on('click', 'li', function () {
  // monitor pause
  ROSLIB.RWTRobotMonitor.prototype.is_paused = true;
  $('#pause-button').hide();
  $('#start-button').show();

  var num = $(this).attr('id').substr(3);
  var data = {};
  data = arrData[Math.abs(parseInt(num, 10) - (arrData.length - 1))];

  var msg = data[0];
  ROSLIB.RWTRobotMonitor.prototype.showHistory(msg);
});
