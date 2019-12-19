Ros.prototype.getSrvList = function (callback, failedCallback) {
  var rwt_srv = new Service({
    ros: this,
    name: '/srv_list',
    serviceType: 'rwt_srv/SrvList'
  });
  var request = new ServiceRequest();
  rwt_srv.callService(request, function (result) {
    callback(result);
  }, function (msg) {
    failedCallback(msg);
  });
};
