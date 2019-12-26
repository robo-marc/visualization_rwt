ROSLIB.Ros.prototype.autoConnect = function () {
    var that = this;
    var value = "ws://" + location.hostname + ":8888/";
    that.connect(value);
};
