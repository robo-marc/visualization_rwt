
$(function () {

  //TODO callback test
  var test2 = function (arg) {
    console.log('hello, ' + arg);
  };
  var test1 = function (arg) {
    console.log(typeof arg);
    arg('abc');
    arg('def');
  };
  test1(test2);

  // service rosparam
  var ros = new ROSLIB.Ros();

  var rosparamOptionList = [
    'list',
    'set',
    'get',
    'load',
    'dump',
    'delete',
  ];

  var rosparamList = {};
  var fileObj = document.getElementById('load');
  // var fileObj = $('#load');
  var fileData = [];

  var columns = [
    // { id: 'list', name: 'List', field: 'list', width: 340, sortable: true },
    // { id: 'value', name: 'Value', field: 'value', width: 340, sortable: true },
    { id: 'list', name: 'List', field: 'list', width: 340 },
    { id: 'value', name: 'Value', field: 'value', width: 340 },
  ];
  var options = {
    enableCellNavigation: true,
    enableColumnReorder: false,
    // autoEdit: true

  };

  var data = [];
  var grid = new Slick.Grid('#myGrid', data, columns, options);

  // initialize screen
  function initScreen() {
    // common
    ros.autoConnect();
    // for dropdown
    getSelectList();
    // clear list
    clearList();
  }

  function getSelectList() {
    $('#rosparam-select').append(rosparamOptionList.map(function (value) {
      return '<option value="' + value + '">' + value + '</option>';
    }).join('\n'));
    $('#rosparam-select').change();
  }

  function gridList(rosparamList) {
    data = [];
    grid.setData(data);
    var i = 0;
    for (var key in rosparamList) {
      data[i] = {
        list: key,
        value: rosparamList[key],
      };
      i++;
    }

    //sort
    data.sort(function (a, b) {
      if (a.list > b.list) {
        return 1;
      } else {
        return -1;
      }
    });

    grid.invalidateAllRows();
    grid.render();

    grid.resizeCanvas();
    grid.autosizeColumns();
    // prevent the delete button from being hidden when switching screens vertically
    grid.resizeCanvas();
  }

  // clear messagebox
  function initMessage() {
    // TODO placeholder
    $('#message').text('System message might be shown here when necessary');
    // $('#message').text('');
  }

  // set messagebox
  function setMessage(message) {
    $('#message').text(message);
  }

  // clear list
  function clearList() {
    for (var key in rosparamList) {
      delete rosparamList[key];
    }
    gridList(rosparamList);
  }

  // to float
  function tryToFloat(value) {
    if ($.isNumeric(value)) {
      return parseFloat(value);
    } else {
      return value;
    }
  }

  //set arg1
  function setArg1List() {
    $('#rosparam-arg1').val('');
    $('#rosparam-arg1-data').empty();
    $('#rosparam-arg2').val('');
    ros.getParams(function (names) {
      names.sort();
      $('#rosparam-arg1-data').append(names.map(function (name) {
        return '<option value="' + name + '">' + name + '</option>';
      }).join('\n'));
    },
      function (message) {
        console.log('getParams failed: %s', message);
      }
    );
  }

  $('#rosparam-arg1').hide();
  $('#rosparam-arg2').hide();
  $('#load').hide();
  $('#folder').hide();

  // file change
  $('#folder').on('click', function (e) {
    e.preventDefault();
    $('#load').click();
  });

  fileObj.addEventListener('change', function (evt) {
    var file = evt.target.files;
    // read text
    var reader = new FileReader();
    reader.readAsText(file[0]);
    $('#rosparam-arg2').val(file[0].name);
    reader.onload = function (ev) {
      fileData = reader.result;
    };
  });

  $('#rosparam-select').change(function () {
    var rosparamOption = $('#rosparam-select').val();
    $('#rosparam-arg2').val('');
    $('#load').val('');
    switch (rosparamOption) {
      case 'list':
        $('#rosparam-arg1').hide();
        $('#rosparam-arg2').hide();
        $('#folder').hide();
        $('#load').hide();
        initMessage();
        break;
      case 'set':
        $('#rosparam-arg1').show();
        $('#rosparam-arg2').show();
        $('#folder').hide();
        $('#load').hide();
        setArg1List();
        initMessage();
        break;
      case 'get':
        $('#rosparam-arg1').show();
        $('#rosparam-arg2').hide();
        $('#folder').hide();
        $('#load').hide();
        setArg1List();
        initMessage();
        break;
      case 'load':
        $('#rosparam-arg1').hide();
        $('#rosparam-arg2').show();
        $('#folder').show();
        $('#load').hide();
        initMessage();
        break;
      case 'dump':
        $('#rosparam-arg1').hide();
        $('#rosparam-arg2').hide();
        $('#folder').hide();
        $('#load').hide();
        initMessage();
        break;
      case 'delete':
        $('#rosparam-arg1').show();
        $('#rosparam-arg2').hide();
        $('#folder').hide();
        $('#load').hide();
        setArg1List();
        initMessage();
        break;
    }
  });

  // get param value
  function getParam(name, param, rosparamList) {
    var defer = $.Deferred();

    param.get(function (value) {
      rosparamList[name] = value;
      defer.resolve();
    });

    return defer.promise();
  }

  $('#execute-button').on('click', function () {
    var rosparamOption = $('#rosparam-select').val();
    var rosparamArg1 = $('#rosparam-arg1').val();
    var rosparamArg2 = $('#rosparam-arg2').val();
    var rosparamArg;
    var requestDefer = $.Deferred();
    var promises = [requestDefer.promise()];

    switch (rosparamOption) {
      case 'list':
        initMessage();
        clearList();
        ros.getParams(
          function (names) {
            names.sort();
            _.each(names, function (name, index) {
              var param = new ROSLIB.Param({
                ros: ros,
                name: name,
              });
              // asynchronous processing
              var promise = getParam(name, param, rosparamList);
              promises.push(promise);
            });
            requestDefer.resolve();

            // When the asynchronous processing is terminated  
            $.when.apply(null, promises).done(function () {
              gridList(rosparamList);
            });
          },
          function (message) {
            //error message
            $('#message').text(message);
            requestDefer.resolve();
          }
        );
        break;
      case 'set':
        initMessage();
        if (rosparamArg1 && rosparamArg2) {
          var paramType = new ROSLIB.Param({
            ros: ros,
            name: rosparamArg1,
          });
          paramType.get(function (value) {
            requestDefer.resolve();
          },
            function (message) {
              $('#message').val(message);
              requestDefer.resolve();
            }
          );

          $.when.apply(null, promises).done(function () {
            // When the asynchronous processing is terminated
            rosparamArg = tryToFloat(rosparamArg2);
            var paramSet = new ROSLIB.Param({
              ros: ros,
              name: rosparamArg1,
            });
            paramSet.set(rosparamArg, function (result) {
              console.log(result);
            },
              function (message) {
                //error message
                $('#message').text(message);
              }
            );
          });
        } else {
          //error message
          $('#message').text('Usage: rosparam set [options] parameter\n'
            + 'rosparam: error: '
            + 'invalid arguments. Please specify a parameter name');
        }
        break;
      case 'get':
        initMessage();
        if (rosparamArg1) {
          $('#rosparam-arg2').val('');
          var paramGet = new ROSLIB.Param({
            ros: ros,
            name: rosparamArg1,
          });
          paramGet.get(function (value) {
            if (value) {
              clearList();
              rosparamList[rosparamArg1] = value;
              gridList(rosparamList);
            } else {
              clearList();
              //error message
              $('#message').text('ERROR: Parameter ['
                + rosparamArg1
                + '] is not set');
            }
          });
        } else {
          clearList();
          //error message
          $('#message').text('Usage: rosparam get [options] parameter\n'
            + 'rosparam: error: '
            + 'invalid arguments. Please specify a parameter name');
        }
        break;
      case 'load':
        initMessage();
        if (rosparamArg2) {
          if (fileData) {
            ros.loadParams(fileData, function (result) {
              console.log(result);
            },
              function (message) {
                console.log('gloadParams failed: %s', message);
                //error message
                $('#message').text(message);
              }
            );
          }
          // }
        } else {
          clearList();
          //error message
          $('#message').text('Usage: rosparam load [options] file [namespace]\n'
            + 'rosparam: error: '
            + 'invalid arguments. Please specify a file name or - for stdin');
        }
        break;
      case 'dump':
        var download = '';
        initMessage();
        ros.dumpParams(function (value) {
          if (value) {
            for (var i = 0; i < value.params.length; i++) {
              var param = value.params[i].replace(/^"(.*)"$/, '$1');
              console.log(param);
              download = download + param;
              console.log(download);
            }
            var link = document.createElement('a');
            link.href = window.URL.createObjectURL(new Blob([download]));
            link.download = 'rosparam.yaml';
            link.click();
          }
        },
          function (message) {
            console.log('dumpParams failed: %s', message);
            //error message
            $('#message').text(message);
          }
        );
        break;
      case 'delete':
        initMessage();
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
          //error message
          $('#message').text('Usage: rosparam delete [options] parameter\n'
            + 'rosparam: error: '
            + 'invalid arguments. Please specify a parameter name');
        }
        break;
    }
  });

  $(window).on('load resize', function () {
    grid.resizeCanvas();
    grid.autosizeColumns();

    // prevent the delete button from being hidden when switching screens vertically
    grid.resizeCanvas();
  });

  initScreen();
});


