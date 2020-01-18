// DiagnosticsDirectory.js

/**
 * @fileOverview a file to define RWTRobotMonitor.DiagnosticsDirectory class
 * @author Ryohei Ueda
 */

/**
 * @class DiagnosticsDirectory
 * @param spec
 */
ROSLIB.DiagnosticsDirectory = function (spec, parent, children) {
  if (typeof parent === 'undefined') {
    parent = null;
  }
  if (typeof children === 'undefined') {
    children = [];
  }

  this.parent = parent;
  this.children = children;
  this.status = spec.status;
  this.name = spec.name;
};

/**
 * lookup the directory whose name equals to `name'. This method digs the children
 * of the directory. If succeeeds to find that, returns the instance
 * of ROSLIB.DiagnosticsDirectory. if failed, returns null.
 * @param name - the name of directory
 */
ROSLIB.DiagnosticsDirectory.prototype.findDirectory = function (name) {
  if (this.name.toString() === name.toString()) {
    return this;
  }
  else if (this.children.length === 0) { // no children
    return null;
  }
  else {
    for (var i = 0; i < this.children.length; i++) {
      var child_result = this.children[i].findDirectory(name);
      if (child_result !== null) {
        return child_result;
      }
    }
    return null;
  }
};

/**
 * add child directory to this directory
 * @param directory - an instance of ROSLIB.DiagnosticsDirectory
 */
ROSLIB.DiagnosticsDirectory.prototype.addChild = function (directory) {
  this.children.push(directory);
  directory.parent = this;
};

/**
 * create a child directory which has this directory as parent
 * @ param name - name of child directory
 */
ROSLIB.DiagnosticsDirectory.prototype.createChild = function (name) {
  var child = new ROSLIB.DiagnosticsDirectory({
    name: name
  }, this);
  this.addChild(child);
  return child;
};

/**
 * register a status to the directory
 * @param status - instance of ROSLIB.DiagnosticsStatus
 */
ROSLIB.DiagnosticsDirectory.prototype.registerStatus = function (status) {
  this.status = status;
  return status;
};

/**
 * return the instance of directory if the directory has error instance 
 * as children.
 */
ROSLIB.DiagnosticsDirectory.prototype.isChildrenHasError = function () {
  if (this.isErrorStatus()) {
    return this;
  }
  else {
    for (var i = 0; i < this.children.length; i++) {
      var child_result = this.children[i].isChildrenHasError();
      if (child_result) {
        return child_result;
      }
    }
  }
};

/**
 * return true if the status registered to the directory has error level.
 */
ROSLIB.DiagnosticsDirectory.prototype.isErrorStatus = function () {
  if (this.status) {
    return this.status.isERROR();
  }
  else {
    return false;
  }
};

/**
 * return full path of the directory
 */
ROSLIB.DiagnosticsDirectory.prototype.fullName = function () {
  var rec = function (target_dir) {
    if (target_dir.parent === null) { // root
      return '';
    }
    else {
      var parent_result = rec(target_dir.parent);
      return parent_result + '/' + target_dir.name;
    }
  };
  return rec(this);
};

/**
 * get the uniq id which is available as html id
 */
ROSLIB.DiagnosticsDirectory.prototype.uniqID = function () {
  var rec = function (target_dir) {
    if (target_dir.parent === null) { // root
      return '';
    }
    else {
      var parent_result = rec(target_dir.parent);
      if (parent_result.length === 0) {
        return target_dir.name;
      }
      else {
        return parent_result + '-' + target_dir.name;
      }
    }
  };
  return rec(this).replace(' ', '-').replace('(', '').replace(')', '').replace('\t', '-');
};


/**
 * get an array of all the directories without root itself
 */
ROSLIB.DiagnosticsDirectory.prototype.getAllDirectoriesWithoutRoot = function () {
  var self = this;
  var directories = self.getAllDirectories();
  _.remove(directories, function (dir) {
    return dir === self;
  });
  return directories;
};

/**
 * get an array of all the directories
 */
ROSLIB.DiagnosticsDirectory.prototype.getAllDirectories = function () {
  var rec = function (target_dir) {
    if (target_dir.children.length === 0) {
      return [target_dir];
    }
    else {
      var result = [];
      for (var i = 0; i < target_dir.children.length; i++) {
        var child_result = rec(target_dir.children[i]);
        result = result.concat(child_result);
      }
      result.push(target_dir);
      return result;
    }
  };
  return rec(this);
};


/**
 * get an array of directories which has `level' such as error, warning and ok.
 */
ROSLIB.DiagnosticsDirectory.prototype.getDirectories = function (level) {
  var rec = function (target_dir) {
    if (target_dir.children.length === 0) {
      if (target_dir.status && target_dir.status.level === level) {
        return [target_dir];
      }
      else {
        return [];
      }
    }
    else {
      var result = [];
      for (var i = 0; i < target_dir.children.length; i++) {
        var child_result = rec(target_dir.children[i]);
        result = result.concat(child_result);
      }
      if (target_dir.status && target_dir.status.level === level) {
        result.push(target_dir);
      }
      return result;
    }
  };
  return rec(this);
};

/**
 * return an array of directories which has error status
 */
ROSLIB.DiagnosticsDirectory.prototype.getErrorDirectories = function () {
  return this.getDirectories(ROSLIB.DiagnosticsStatus.LEVEL.ERROR);
};

/**
 * return an array of directories which has warn status
 */
ROSLIB.DiagnosticsDirectory.prototype.getWarnDirectories = function () {
  return this.getDirectories(ROSLIB.DiagnosticsStatus.LEVEL.WARN);
};

/**
 * return an array of directories which has ok status
 */
ROSLIB.DiagnosticsDirectory.prototype.getOkDirectories = function () {
  return this.getDirectories(ROSLIB.DiagnosticsStatus.LEVEL.OK);
};

/**
 * look up the directory by name throught directory tree return the instance of the directory
 */
ROSLIB.DiagnosticsDirectory.prototype.findByName = function (name) {
  if (this.status && this.status.name.toString() === name.toString()) {
    return this;
  }
  else {
    for (var i = 0; i < this.children.length; i++) {
      var child_result = this.children[i].findByName(name);
      if (child_result) {
        return child_result;
      }
    }
    return null;
  }
};

// TODO delete
/**
 * return html to show icon suitable for error status of the directory
 */
ROSLIB.DiagnosticsDirectory.prototype.getIconHTML = function () {
  if (this.status.level === ROSLIB.DiagnosticsStatus.LEVEL.OK) {
    return '<span class="glyphicon-ok glyphicon"></span>';
  }
  else if (this.status.level === ROSLIB.DiagnosticsStatus.LEVEL.WARN) {
    return '<span class="glyphicon-exclamation-sign glyphicon"></span>';
  }
  else if (this.status.level === ROSLIB.DiagnosticsStatus.LEVEL.ERROR) {
    return '<span class="glyphicon-minus-sign glyphicon"></span>';
  }
};

/**
 * return html to show icon suitable for error status of the directory
 */
ROSLIB.DiagnosticsDirectory.prototype.getIconHTML2 = function () {
  if (this.status.level === ROSLIB.DiagnosticsStatus.LEVEL.OK) {
    // return '<span class="glyphicon-ok glyphicon"></span>';
    return 'ok';
  }
  else if (this.status.level === ROSLIB.DiagnosticsStatus.LEVEL.WARN) {
    // return '<span class="glyphicon-exclamation-sign glyphicon"></span>';
    return 'warn';
  }
  else if (this.status.level === ROSLIB.DiagnosticsStatus.LEVEL.ERROR) {
    // return '<span class="glyphicon-minus-sign glyphicon"></span>';
    return 'error';
  }
};

// TODO delete
/**
 * return html of icon to show this directory has child
 */
ROSLIB.DiagnosticsDirectory.prototype.getCollapseIconHTML = function () {
  if (this.children.length !== 0) {
    return '<span class="glyphicon-chevron-right glyphicon"></span>';
  }
  else {
    return '';
  }
};

/**
 * return html of icon to show this directory has child
 */
ROSLIB.DiagnosticsDirectory.prototype.getCollapseIconHTML2 = function () {
  if (this.children.length !== 0) {
    return '<span class="glyphicon-chevron-right glyphicon"></span>';
  }
  else {
    return '';
  }
};


/**
 * return true if the directory has any children
 */
ROSLIB.DiagnosticsDirectory.prototype.hasChildren = function () {
  return this.children.length !== 0;
};

// DiagnosticsHistory

/**
 * @fileOverview a file to define RWTRobotMonitor.DiagnosticsHistory class.
 * @author Ryohei Ueda
 */

/**
 * @class DiagnosticsHistory
 * @param spec
 */

ROSLIB.DiagnosticsHistory = function(spec) {
  if (typeof spec === 'undefined') {
    spec = {};
  }
  this.root = new ROSLIB.DiagnosticsDirectory({name: 'root'});
};

/**
 * adding a status to the history.
 * @param status - instance of DiagnosticsStatus
 */
ROSLIB.DiagnosticsHistory.prototype.registerStatus = function(status) {
  // parse paths
  // lookup directory to insert into
  var parent_dir = this.root;
  for (var i = 0; i < status.path.length; i++) {
    var the_dir = parent_dir.findDirectory(status.path[i]);
    if (the_dir === null) {
      the_dir = parent_dir.createChild(status.path[i]);
    }
    parent_dir = the_dir;
  }
  // finally, parent_dir should point to the directory
  // the status should be inserted into
  parent_dir.registerStatus(status);
  return parent_dir;
};


// DiagnosticsStatus

/**
 * @fileOverview a file to define RWTRobotMonitor.DiagnosticsStatus class.
 * @author Ryohei Ueda
 */


/**
 * @class DiagnosticsStatus
 * @param spec
 */
ROSLIB.DiagnosticsStatus = function (spec) {
  if (typeof spec === 'undefined') {
    spec = {};
  }
  this.name = spec.name;
  this.message = spec.message;
  this.level = spec.level;
  this.hardware_id = spec.hardware_id;
  this.values = {};
  this.stamp = spec.timestamp;
  for (var i = 0; i < spec.values.length; i++) {
    this.values[spec.values[i].key] = spec.values[i].value;
  }

  // parsing name
  // name has a directory separated by /
  this.path = _.filter(this.name.split('/'), function (str) {
    return str.toString() !== ''.toString();
  });
};

ROSLIB.DiagnosticsStatus.LEVEL = {
  OK: 0,
  WARN: 1,
  ERROR: 2
};

/**
 * return true if the level is OK
 */
ROSLIB.DiagnosticsStatus.prototype.isOK = function () {
  return this.level === ROSLIB.DiagnosticsStatus.LEVEL.OK;
};

/**
 * return true if the level is WARN
 */
ROSLIB.DiagnosticsStatus.prototype.isWARN = function () {
  return this.level === ROSLIB.DiagnosticsStatus.LEVEL.WARN;
};

/**
 * return true if the level is ERROR
 */
ROSLIB.DiagnosticsStatus.prototype.isERROR = function () {
  return this.level === ROSLIB.DiagnosticsStatus.LEVEL.ERROR;
};

/**
 * create DiagnosticsStatus instances from DiagnosticArray
 */
ROSLIB.DiagnosticsStatus.createFromArray = function (msg) {
  var header = msg.header;
  var header_stamp = ROSLIB.Time.fromROSMsg(header);
  return _.map(msg.status, function (status) {
    return new ROSLIB.DiagnosticsStatus({
      timestamp: header_stamp,
      name: status.name,
      message: status.message,
      hardware_id: status.hardware_id,
      level: status.level,
      values: status.values
    });
  });
};

/**
 * return the level as string
 */

ROSLIB.DiagnosticsStatus.prototype.levelString = function () {
  if (this.isERROR()) {
    return 'ERROR';
  }
  else if (this.isWARN()) {
    return 'WARNING';
  }
  else if (this.isOK()) {
    return 'OK';
  }
};

// RobotMonitor.js

/**
 * @fileOverview a file to define RWTRobotMonitor class.
 * @author F-ROSROBO
 */

// previous data
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
 * update table view  error warn
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
 * update table view all
 */
ROSLIB.RWTRobotMonitor.prototype.updateAllTable = function () {
  // check collapsed list from last time
  var collapseIdList = [];

  $('#all-table .collapse').each(function () {
    collapseIdList.push($(this).attr('id'));
  });
  // TODO test
  console.log('----- allTable collapseIdList ----');
  console.log(collapseIdList);

  //delete table
  $('#all-table tr:gt(0)').remove();
  $('#all-table tr:gt(0)').size();

  // return jquery object
  var rec = function (directory, indent, parentId, toggleParent, displayParent) {

    console.log('----- directory ----');
    console.log(directory);

    // indent
    var leaf = ' leaf leaf' + indent;
    // toggle (expand/collapse)
    var toggle = '';
    // display
    var display = '';

    // toggle check 
    if (directory.children.length !== 0) {
      for (var k = 0; k < collapseIdList.length; k++) {
        if (collapseIdList[k].toString() === directory.uniqID().toString()) {
          toggle = ' collapse';
          break;
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

    if (directory.children.length === 0) {
      // last child
      console.log('----- allTableTree_child ----');
      console.log(allTableTree);
      return allTableTree;
    } else {
      indent++;
      for (var i = 0; i < directory.children.length; i++) {
        var the_child = directory.children[i];
        var the_result = rec(the_child, indent, directory.uniqID(), toggle, display);
        allTableTree.after(the_result);
      }
      // TODO test
      console.log('----- allTableTree_parent ----');
      console.log(allTableTree);
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

  // var history = [];
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

  $('tr.expand').on('click', function () {

    console.log('---- toggle result collapse ----');
    console.log('---- $(this) ----');
    console.log($(this));

    toggleAllList(this);
  });

  $('tr.collapse').on('click', function () {

    console.log('---- toggle result expand ----');
    console.log('---- $(this) ----');
    console.log($(this));

    toggleAllList(this);
  });

  // toggle all list (expand/collapse)
  function toggleAllList(parent) {

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
        var result = rec($(the_child).attr('id'), collapse);
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
