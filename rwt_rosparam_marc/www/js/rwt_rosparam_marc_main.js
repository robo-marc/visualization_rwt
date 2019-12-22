
$(function () {

  // service rosparam
  var ros = new ROSLIB.Ros();
  ros.install_config_button('config-button');

  var rosparamOptionList = [
    'list',
    'set',
    'get',
    'load',
    'dump',
    'delete',
  ];

  $('#rosparam-select').append(rosparamOptionList.map(function (value) {
    return '<option value=' + value + '>' + value + '</option>';
  }).join('\n'));
  $('#rosparam-select').change();

  //TODO test dummy
  var rosparamArg1List = [
    '/rosapi/params_glob',
    '/rosbridge_websocket/max_message_size',
    '/rosbridge_websocket/port',
    '/rosdistro',
  ];
  $('#rosparam-arg1').append(rosparamArg1List.map(function (value) {
    return '<option value=' + value + '>' + value + '</option>';
  }).join('\n'));
  $('#rosparam-arg1').change();

  $('#rosparam-arg1').hide();
  $('#rosparam-arg2').hide();

  $('#rosparam-select').click(function () {
    var rosparamOption = $('#rosparam-select').val();
    switch (rosparamOption) {
      case 'list':
        $('#rosparam-arg1').hide();
        $('#rosparam-arg2').hide();
        break;
      case 'set':
        $('#rosparam-arg1').show();
        $('#rosparam-arg2').show();
        break;
      case 'get':
        $('#rosparam-arg1').show();
        $('#rosparam-arg2').show();
        break;
      case 'load':
        $('#rosparam-arg1').show();
        $('#rosparam-arg2').show();
        break;
      case 'dump':
        $('#rosparam-arg1').show();
        $('#rosparam-arg2').show();
        break;
      case 'delete':
        $('#rosparam-arg1').show();
        $('#rosparam-arg2').hide();
        break;
    }

  });

  $('#execute-button').on('click', function () {
    var rosparamOption = $('#rosparam-select').val();
    var rosparamArg1 = $('#rosparam-arg1').val();
    var rosparamArg2 = $('#rosparam-arg2').val();

    switch (rosparamOption) {
      case 'list':
        ros.getParams(function (names) {
          // var rosparamList = names;
          // var callback_result = failedCallback;
          _.each(names, function (name, index) {
            console.log(name);
            var param = new ROSLIB.Param({
              ros: ros,
              name: name,
            });
            param.get(function (value) {
              console.log(value);
            });
          });
        },
          function (message) {
            console.log('getParams failed: %s', message);
          }
        );

        break;
      case 'set':
        if (rosparamArg1 && rosparamArg2) {
          var paramSet = new ROSLIB.Param({
            ros: ros,
            name: rosparamArg1,
          });
          paramSet.set(rosparamArg2, function (result) {
            console.log(result);
          },
            function (message) {
              console.log('setParams failed: %s', message);
            }
          );
        }
        else {
          //TODO ErrorMessage
          console.log('Usage: rosparam set [options] parameter value\n' +
            'rosparam: error: ' +
            'invalid arguments.Please specify a parameter name');
        }
        break;
      case 'get':
        if (rosparamArg1) {
          $('#rosparam-arg2').val('');
          var paramGet = new ROSLIB.Param({
            ros: ros,
            name: rosparamArg1,
          });
          paramGet.get(function (value) {
            console.log(value);
            $('#rosparam-arg2').val(value);
          });
        }
        else {
          //TODO ErrorMessage
          console.log('Usage: rosparam get [options] parameter\n' +
            'rosparam: error: ' +
            'invalid arguments. Please specify a parameter name');
        }
        break;
      case 'load':

        // TODO rwt_topic_marc test getService
        ros.getTopics(function (topics) {
          console.log(topics);
        },
          function (message) {
            console.log('getTopics failed: %s', message);
          }
        );
        // ros.getTopicType(rosparamArg2, function (result) {
        //   console.log(result);
        // },
        //   function (message) {
        //     console.log('TopicsForType failed: %s', message);
        //   }
        // );
        // ros.getMessageDetails(rosparamArg2, function (types) {
        //   console.log(types);
        // },
        //   function (message) {
        //     console.log('getMessage failed: %s', message);
        //   }
        // );
        // 例:plotを参考にする
        var subscriber = new ROSLIB.Topic({
          ros: ros,
          name: '/sin',
          messageType: 'std_msgs/Float64'
        });
        subscriber.subscribe(function (msg) {
          console.log(msg);
        });


        // TODO rwt_action_marc test getService
        // ros.getMessageDetails(rosparamArg2, function (types) {
        //   console.log(types);
        // },
        //   function (message) {
        //     console.log('getMessage failed: %s', message);
        //   }
        // );

        // // TODO rwt_srv_marc test getService
        // ros.getServices(function (result) {
        //   console.log(result);
        // },
        //   function (message) {
        //     console.log('getServices failed: %s', message);
        //   }
        // );
        // ros.getServiceRequestDetails(rosparamArg2, function (types) {
        //   console.log(types);
        // },
        //   function (message) {
        //     console.log('getServiceRequest failed: %s', message);
        //   }
        // );
        // ros.getServiceResponseDetails(rosparamArg2, function (types) {
        //   console.log(types);
        // },
        //   function (message) {
        //     console.log('getServiceResponse failed: %s', message);
        //   }
        // );

        break;
      case 'dump':
        // var param = new ROSLIB.Param({
        //   ros: ros,
        //   name: rosparamArg2,
        // })
        ros.dumpParams(function (value) {
          console.log(value);
        });
        break;
      case 'delete':
        if (rosparamArg1) {
          var paramDel = new ROSLIB.Param({
            ros: ros,
            name: rosparamArg1,
          });
          paramDel.delete(function (result) {
            console.log(result);
          });
        }
        else {
          //TODO ErrorMessage
          console.log('Usage: rosparam delete [options] parameter\n' +
            'rosparam: error: ' +
            'invalid arguments. Please specify a parameter name');
        }

        break;
    }
  });

});


