var data = [];
var searchString = '';


$(function () {

  // subscribe topic
  var ros = new ROSLIB.Ros();
  ros.install_config_button('config-button');
  var serviceMap = new Map();

  //getSrvList request
  function requestService() {
    ros.getSrvList(
      function (result) {
        var objList = [];
        _.each(result.message,
          function (msg, index) {
            var replace = msg.split('\'').join('\"');
            var obj = JSON.parse(replace);
            objList.push(obj);
          });
        console.log(objList);
        selectService(objList);
      },
      function (mes) {
        console.log(mes);
      }
    );
  }

  requestService();

  // ツリー用追加ロジック/////////////////////////////////////////////
  var TaskNameFormatter = function (row, cell, value, columnDef, dataContext) {
    if (value === null || value === undefined || dataContext === undefined) { return ''; }

    value = value.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
    var spacer = '<span style="display:inline-block;height:1px;width:' + (15 * dataContext['indent']) + 'px"></span>';
    var idx = dataView.getIdxById(dataContext.id);
    if (data[idx + 1] && data[idx + 1].indent > data[idx].indent) {
      if (dataContext._collapsed) {
        return spacer + '<span class="toggle expand"></span>&nbsp;' + value;
      } else {
        return spacer + '<span class="toggle collapse"></span>&nbsp;' + value;
      }
    } else {
      return spacer + '<span class="toggle"></span>&nbsp;' + value;
    }
  };
  ///////////////////////////////////////////////////////////////////

  // ★グリッドアイテム取得/////////////////////////////////////////
  var dataView;
  var grid;
  var columns = [
    { id: 'tree', name: 'Tree', field: 'tree', width: 200, minWidth: 20, maxWidth: 300, formatter: TaskNameFormatter },
    { id: 'type', name: 'Type', field: 'type', width: 260, minWidth: 20, maxWidth: 900, },
    { id: 'path', name: 'Path', field: 'path', width: 260, minWidth: 20, maxWidth: 300, },
    //削除ボタンを表示するためのカラムを定義
    {
      id: 'remove', name: 'Remove', field: 'remove', width: 100, minWidth: 20, maxWidth: 200, formatter: function () {
        return '<button class="delete-button"><span class="glyphicon glyphicon-minus-sign" aria-hidden="true"></span></button>';
      }
    }
  ];

  // ツリー用追加ロジック/////////////////////////////////

  function myFilter(item) {
    if (item.parent !== null) {
      var parent = data[item.parent];

      while (parent) {
        if (parent._collapsed || (searchString !== '')) {
          return false;
        }

        parent = data[parent.parent];
      }
    }

    return true;

  }
  ///////////////////////////////////////////////////////////

  // serviceのリストを表示するリストボックス
  function selectService(objList) {
    var firstAddServiceStr = '';
    for (var i = 0; i < objList.length; i++) {
      var recordData = objList[i];
      for (var j = 0; j < recordData.length; j++) {
        var serviceElement = recordData[j];
        var serviceStr = serviceElement.substring(0, serviceElement.indexOf('/'));
        var messageStr = serviceElement.substring(serviceElement.indexOf('/') + 1);

        if (serviceMap.has(serviceStr)) {
          serviceMap.get(serviceStr).push(messageStr);
        } else {
          if (0 === serviceMap.size) {
            firstAddServiceStr = serviceStr;
          }
          var mapValue = [messageStr];
          serviceMap.set(serviceStr, mapValue);
          $('#service-select').append('<option value="' + serviceStr + '">' + serviceStr + '</option>');
        }
      }
    }
    ////// 不要？
    var messageList = serviceMap.get(firstAddServiceStr);
    for (var k = 0; k < messageList.length; k++) {
      $('#message-select').append('<option value="' + messageList[k] + '">' + messageList[k] + '</option>');
    }
  }
  ///////

  $('#service-select').on('change', function () {
    var selectedValue = $('#service-select').val();
    $('#message-select').empty();
    var messageList = serviceMap.get(selectedValue);
    for (var k = 0; k < messageList.length; k++) {
      $('#message-select').append('<option value="' + messageList[k] + '">' + messageList[k] + '</option>');
    }
  });



  $('#add-service-button').on('click', function () {
    var indent = 0;
    var parents = [];
    var serviceName = $('#service-select').val();
    var messageName = $('#message-select').val();
    var typeName = serviceName + '/' + messageName;

    var requestDefer = $.Deferred();
    var responseDefer = $.Deferred();
    var promises = [requestDefer.promise(), responseDefer.promise()];

    ros.getServiceRequestDetails(typeName,
      function (types) {
        console.log('got request detail: ' + types);

        var parent;
        var reqData = [];
        reqData.push({
          indent: 0,
          parent: null,
          tree: 'Request',
          type: typeName,
          path: typeName,
          remove: '',
        });

        // TODO: push
        var typedefsData = types['typedefs'];
        var childReqData = typedefsData[0];
        var fieldnames = childReqData['fieldnames'];
        var fieldtypes = childReqData['fieldtypes'];
        for (var i = 0; i < fieldnames.length; i++) {
          reqData.push({
            indent: 1,
            parent: 0,
            tree: fieldnames[i],
            type: fieldtypes[i],
            path: typeName + '/' + fieldnames[i],
            remove: '',
          });
        }
        requestDefer.resolve({ key: 'request', value: reqData });
      },
      function (message) {
        console.log('getParams failed: %s', message);
        requestDefer.resolve();
      }
    );

    ros.getServiceResponseDetails(typeName,
      function (types) {
        console.log('got response detail: ' + types);

        var resData = [];
        resData.push({
          indent: 0,
          parent: 1,
          tree: 'Response',
          type: typeName,
          path: typeName,
          remove: '',
        });

        // TODO: push

        responseDefer.resolve({ key: 'response', value: resData });
      },
      function (message) {
        console.log('getParams failed: %s', message);
        responseDefer.resolve();
      }
    );

    $.when.apply(null, promises).done(function () {
      // 非同期処理が全部終わったときの処理

      var reqData;
      var resData;
      for (var i = 0; i < arguments.length; i++) { // arguments は forEach() 使えない
        var key = arguments[i].key;
        var value = arguments[i].value;
        console.log(key);

        if (key === 'request') {
          reqData = value;
        } else {
          resData = value;
        }
      }

      var mergedData = reqData.concat(resData);

      var itemCount = dataView.getLength();
      var lastItem = dataView.getItem(itemCount - 1) || { id: 0 };
      var nextId = lastItem['id'] + 1;
      dataView.beginUpdate();
      _.each(mergedData, function (item, index) {
        item['id'] = nextId + index;
        dataView.addItem(item);
      });
      dataView.endUpdate();
      // grid = new Slick.Grid("#myGrid", mergedData, columns);
    });
  });

  //★ initialize the model
  dataView = new Slick.Data.DataView({ inlineFilters: true });
  dataView.setItems(data);
  dataView.setFilter(myFilter);

  //★ initialize the grid
  grid = new Slick.Grid('#myGrid', dataView, columns);

  //★
  grid.onClick.subscribe(function (e, args) {
    var $target = $(e.target);

    // ツリー　開く/閉じる　クリックイベントハンドラ
    if ($target.hasClass('toggle')) {
      var item = dataView.getItem(args.row);
      if (item) {
        if (!item._collapsed) {
          item._collapsed = true;
        } else {
          item._collapsed = false;
        }

        dataView.updateItem(item.id, item);
      }
      e.stopImmediatePropagation();
    }

    // 削除クリックイベントハンドラ
    else if ($target.parent().hasClass('delete-button')
      || $target.hasClass('delete-button')) {
      var items = dataView.getItem(args.row);
      if (items) {
        dataView.beginUpdate();
        dataView.deleteItem(items.id);
        dataView.endUpdate();
      }
      // data.splice(args.row, 1);
      // grid.invalidate();
    }
  });

  //★ wire up model events to drive the grid
  dataView.onRowCountChanged.subscribe(function (e, args) {
    grid.updateRowCount();
    grid.render();
  });

  dataView.onRowsChanged.subscribe(function (e, args) {
    grid.invalidateRows(args.rows);
    grid.render();
  });
});


