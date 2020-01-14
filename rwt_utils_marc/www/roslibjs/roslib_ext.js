/**
 * 
 */
ROSLIB.Ros.prototype.getSrvList = function (callback, failedCallback) {
  var client = new ROSLIB.Service({
    ros: this,
    name: '/srv_list',
    serviceType: 'rwt_srv_marc/SrvList'
  });

  var request = new ROSLIB.ServiceRequest();
  client.callService(request,
    function (result) {
      callback(result);
    },
    function (message) {
      failedCallback(message);
    }
  );
};

/**
 *
 */
ROSLIB.Ros.prototype.getActionList = function (callback, failedCallback) {
  var client = new ROSLIB.Service({
    ros: this,
    name: '/action_list',
    serviceType: 'rwt_action_marc/ActionList'
  });

  var request = new ROSLIB.ServiceRequest();
  client.callService(request,
    function (result) {
      callback(result);
    },
    function (message) {
      failedCallback(message);
    }
  );
};

/**
 * Dump ROS parameter.
 * 
 * @param callback - function with params:
 *   * details - Array of the parameters
 */
ROSLIB.Ros.prototype.dumpParams = function (callback, failedCallback) {
  var client = new ROSLIB.Service({
    ros: this,
    name: '/dump_params',
    serviceType: 'rwt_rosparam_marc/DumpParams'
  });
  var request = new ROSLIB.ServiceRequest();
  if (typeof failedCallback === 'function') {
    client.callService(request,
      function (result) {
        callback(result);
      },
      function (message) {
        failedCallback(message);
      }
    );
  } else {
    client.callService(request, function (result) {
      callback(result);
    });
  }
};

/**
 * Load ROS parameter.
 * 
 * @param callback - function with params:
 * @param params - String of parameters
 */
ROSLIB.Ros.prototype.loadParams = function (params, callback, failedCallback) {
  var client = new ROSLIB.Service({
    ros: this,
    name: '/load_params',
    serviceType: 'rwt_rosparam_marc/LoadParams'
  });

  var request = new ROSLIB.ServiceRequest({
    name: this.name,
    // params: JSON.stringify(params)
    params: params
  });
  if (typeof failedCallback === 'function') {
    client.callService(request,
      function (result) {
        callback(result);
      },
      function (message) {
        failedCallback(message);
      }
    );
  } else {
    client.callService(request, function (result) {
      callback(result);
    });
  }
};

// TODO del
/**
 * Retrieves Hz Bandwidth in ROS from Topic
 * @class Ros
 * @param topicName topic name to find:
 * @param topicType topic type to find:
 */
ROSLIB.Ros.prototype.getTopicInfo = function (topicName, topicType, callback, failedCallback) {
  var client = new ROSLIB.Service({
    ros: this,
    name: '/topic_hz_bw',
    serviceType: 'rwt_rostopic_marc/TopicHzBw'
  });

  var request = new ROSLIB.ServiceRequest({
    name: topicName,
    type: topicType
  });
  if (typeof failedCallback === 'function') {
    client.callService(request,
      function (result) {
        callback(result);
      },
      function (message) {
        failedCallback(message);
      }
    );
  } else {
    client.callService(request, function (result) {
      callback(result);
    });
  }
};

/**
 * Start monitoring to retrieves Hz Bandwidth in ROS from Topic
 * @class Ros
 * @param topicName topic name to find:
 * @param topicType topic type to find:
 */
ROSLIB.Ros.prototype.startMonitoring = function (topicName, topicType, callback, failedCallback) {
  var client = new ROSLIB.Service({
    ros: this,
    name: '/start_monitoring',
    serviceType: 'rwt_rostopic_marc/StartMonitoring'
  });

  var request = new ROSLIB.ServiceRequest({
    name: topicName,
    type: topicType
  });
  if (typeof failedCallback === 'function') {
    client.callService(request,
      function (result) {
        callback(result);
      },
      function (message) {
        failedCallback(message);
      }
    );
  } else {
    client.callService(request, function (result) {
      callback(result);
    });
  }
};

/**
 * Stop monitoring to retrieves Hz Bandwidth in ROS from Topic
 * @class Ros
 * @param topicName topic name to find:
 * @param topicType topic type to find:
 */
ROSLIB.Ros.prototype.stopMonitoring = function (topicName, topicType, callback, failedCallback) {
  var client = new ROSLIB.Service({
    ros: this,
    name: '/stop_monitoring',
    serviceType: 'rwt_rostopic_marc/StopMonitoring'
  });

  var request = new ROSLIB.ServiceRequest({
    name: topicName,
    type: topicType
  });
  if (typeof failedCallback === 'function') {
    client.callService(request,
      function (result) {
        callback(result);
      },
      function (message) {
        failedCallback(message);
      }
    );
  } else {
    client.callService(request, function (result) {
      callback(result);
    });
  }
};

/**
 * Retrieves Hz Bandwidth in ROS from Topic
 * @class Ros
 */
ROSLIB.Ros.prototype.getMonitoringInfo = function (callback, failedCallback) {
  var client = new ROSLIB.Service({
    ros: this,
    name: '/get_monitoring_info',
    serviceType: 'rwt_rostopic_marc/GetMonitoringInfo'
  });

  var request = new ROSLIB.ServiceRequest(
  );
  if (typeof failedCallback === 'function') {
    client.callService(request,
      function (result) {
        callback(result);
      },
      function (message) {
        failedCallback(message);
      }
    );
  } else {
    client.callService(request, function (result) {
      callback(result);
    });
  }
};

/**
 * Represents a time whith seconds and nanoseconds
 * @class Time
 * @param spec - a dictionary which include nsecs and secs as the keys.
 * @property secs {Integer} seconds
 * @property nsecscs {Integer} nanoseconds (10^-9)
 */
ROSLIB.Time = function (spec) {
  this.nsecs = Math.floor((spec || {}).nsecs || 0);
  this.secs = Math.floor((spec || {}).secs || 0);
};

ROSLIB.Time.now = function () {
  var now = new Date();
  var msec = now.getTime();
  return new ROSLIB.Time({
    secs: Math.floor(msec / 1000),
    nsecs: (msec % 1000) * 1000000
  });
};

ROSLIB.Time.prototype.toSec = function () {
  return this.secs + this.nsecs / 1000000000.0;
};

ROSLIB.Time.prototype.toMillSec = function () {
  return this.secs * 1000 + this.nsecs / 1000000.0;
};

ROSLIB.Time.prototype.add = function (another) {
  var sec_added = this.secs + another.secs;
  var nsec_added = this.nsecs + another.nsecs;
  if (nsec_added > 1000000000) {
    sec_added = sec_added + 1;
    nsec_added = nsec_added - 1000000000;
  }
  return new ROSLIB.Time({
    secs: sec_added,
    nsecs: nsec_added
  });
};

ROSLIB.Time.prototype.substract = function (another) {
  var sec_diff = this.secs - another.secs;
  var nsec_diff = this.nsecs - another.nsecs;
  if (nsec_diff < 0) {
    sec_diff = sec_diff - 1;
    nsec_diff = 1000000000 + nsec_diff;
  }
  return new ROSLIB.Time({
    secs: sec_diff,
    nsecs: nsec_diff
  });
};

ROSLIB.Time.prototype.equal = function (another) {
  var diff = this.substract(another);
  return ((diff.secs === 0) && (diff.nsecs === 0));
};

/**
 * Converts a JSON-ized message of stamp into ROSLIB.Time
 * @param msg - a message of stamp.
 */
ROSLIB.Time.fromROSMsg = function (msg) {
  return new ROSLIB.Time({ secs: msg.secs, nsecs: msg.nsecs });
};

ROSLIB.Time.fromSec = function (sec) {
  return new ROSLIB.Time({ secs: sec, nsecs: 0 });
};

/**
 * Converts ROSLIB.Time into Date object
 */
ROSLIB.Time.prototype.toDate = function () {
  var d = new Date();
  d.setTime(this.secs * 1000 + this.nsecs / 1000000);
  return d;
};

/**
 * Converts Date object into ROSLIB.Time
 */
ROSLIB.Time.fromDate = function (date) {
  var msec = date.getTime();
  return new ROSLIB.Time({
    secs: Math.floor(msec / 1000),
    nsecs: (msec % 1000) * 1000000
  });
};
