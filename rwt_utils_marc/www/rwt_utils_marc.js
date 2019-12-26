ROSLIB.Ros.prototype.autoConnect = function () {
    var value = "ws://" + location.hostname + ":8888/";
    this.connect(value);
};
