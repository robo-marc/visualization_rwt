var dataView;
dataView = new Slick.Data.DataView({ inlineFilters: true });

var data = [];
var searchString = '';

var resParentId;
var reqParentId;

$(function () {

  var grid;

  // subscribe topic
  var ros = new ROSLIB.Ros();
  ros.autoConnect();

  var serviceMap = {};

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
        listDataAcquisition(objList);
      },
      function (mes) {
        console.log(mes);
      }
    );
  }

  requestService();

  // ツリー用追加ロジック/////////////////////////////////////////////
  var treeFormatter = function (row, cell, value, columnDef, dataContext) {
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
  //削除ボタンを表示するためのカラムを定義
  var removeButtonFormatter = function (row, cell, value, columnDef, dataContext) {
    if (value === null || value === undefined || dataContext === undefined) { return ''; }
    var parentId = dataContext.parent;
    if (parentId === null) {
      return '<button class="delete-button"><span class="glyphicon glyphicon-minus-sign" aria-hidden="true"></span></button>';
    }
    return '';
  };

  // ★グリッドアイテム取得/////////////////////////////////////////
  var columns = [
    { id: 'tree', name: 'Tree', field: 'tree', width: 200, minWidth: 20, maxWidth: 300, formatter: treeFormatter },
    { id: 'type', name: 'Type', field: 'type', width: 260, minWidth: 20, maxWidth: 900, },
    { id: 'path', name: 'Path', field: 'path', width: 260, minWidth: 20, maxWidth: 900, },
    { id: 'remove', name: 'Remove', field: 'remove', width: 100, minWidth: 20, maxWidth: 200, formatter: removeButtonFormatter }
  ];

  // ツリー用追加ロジック/////////////////////////////////

  function myFilter(item) {
    if (item.parent !== null) {
      var parent = dataView.getItemById(item.parent);

      while (parent) {
        if (parent._collapsed) {
          return false;
        }

        parent = dataView.getItemById(parent.parent);
      }
    }

    return true;

  }
  ///////////////////////////////////////////////////////////

  // serviceのリストを表示するリストボックス
  function listDataAcquisition(objList) {
    objList.sort();
    for (var i = 0; i < objList.length; i++) {
      var recordData = objList[i];
      for (var j = 0; j < recordData.length; j++) {
        var serviceElement = recordData[j];
        var serviceStr = serviceElement.substring(0, serviceElement.indexOf('/'));
        var messageStr = serviceElement.substring(serviceElement.indexOf('/') + 1);

        if (serviceStr in serviceMap) {
          serviceMap[serviceStr].push(messageStr);
        } else {
          var mapValue = [messageStr];
          serviceMap[serviceStr] = mapValue;
        }
      }
    }
    buildServiceSelect(Object.keys(serviceMap));
  }

  function buildServiceSelect(packageList) {
    var optionsHtml = '';

    // packageList.sort();
    for (var i = 0; i < packageList.length; i++) {
      optionsHtml += '<option value="' + packageList[i] + '">' + packageList[i] + '</option>';
    }

    var $select = $('#service-select');
    $select.empty();
    $select.append(optionsHtml);
    $select.change();
  }

  $('#service-select').on('change', function () {
    var selectedValue = $('#service-select').val();
    var messageList = serviceMap[selectedValue];
    var optionsHtml = '';

    // messageList.sort();
    for (var i = 0; i < messageList.length; i++) {
      optionsHtml += '<option value="' + messageList[i] + '">' + messageList[i] + '</option>';
    }

    var $select = $('#message-select');
    $select.empty();
    $select.append(optionsHtml);
    $select.change();
  });

  var resId = 0;
  var reqId = 1;

  $('#add-button').on('click', function () {
    var serviceName = $('#service-select').val();
    var messageName = $('#message-select').val();
    var typeName = serviceName + '/' + messageName;

    var requestDefer = $.Deferred();
    var responseDefer = $.Deferred();
    var promises = [requestDefer.promise(), responseDefer.promise()];

    var reqData = [];
    ros.getServiceRequestDetails(typeName,
      function (types) {

        var reqDataRetention = ({
          id: reqId,
          indent: 0,
          parent: null,
          tree: 'Request',
          type: typeName,
          path: typeName,
          remove: '',
          _collapsed: true,
        });
        reqParentId = reqId;
        reqId = reqId + 2;
        var reqChildren = [];
        // TODO: push
        var typedefsData = types['typedefs'];
        var childReqData = typedefsData[0];
        var fieldarraylen = childReqData['fieldarraylen'];
        var fieldnames = childReqData['fieldnames'];
        var fieldtypes = childReqData['fieldtypes'];
        for (var i = 0; i < fieldnames.length; i++) {
          var len = fieldarraylen[i];
          reqData.push({
            id: reqId,
            indent: 1,
            fieldarraylen: fieldarraylen[i],
            parent: reqParentId,
            tree: fieldnames[i],
            type: fieldtypes[i] + (len !== -1 ? '[]' : ''),
            path: typeName + '/' + fieldnames[i],
            remove: '',
          });
          reqChildren.push(reqId);
          reqId = reqId + 2;
        }
        reqDataRetention.children = reqChildren;
        reqData.push(reqDataRetention);
        requestDefer.resolve({ key: 'request', value: reqData });
      },
      function (message) {
        console.log('getParams failed: %s', message);
        requestDefer.resolve();
      }
    );
    var resData = [];
    ros.getServiceResponseDetails(typeName,
      function (types) {
        var resDataRetention = ({
          id: resId,
          indent: 0,
          parent: null,
          tree: 'Response',
          type: typeName,
          path: typeName,
          remove: '',
          _collapsed: true,
        });

        resParentId = resId;
        resId = resId + 2;
        var resChildren = [];
        // TODO: push★
        var typedefsData = types['typedefs'];
        var childReqData = typedefsData[0];
        var fieldarraylen = childReqData['fieldarraylen'];
        var fieldnames = childReqData['fieldnames'];
        var fieldtypes = childReqData['fieldtypes'];
        for (var i = 0; i < fieldnames.length; i++) {
          var len = fieldarraylen[i];
          resData.push({
            id: resId,
            indent: 1,
            fieldarraylen: fieldarraylen[i],
            parent: resParentId,
            tree: fieldnames[i],
            type: fieldtypes[i] + (len !== -1 ? '[]' : ''),
            path: typeName + '/' + fieldnames[i],
            remove: '',
          });
          resChildren.push(resId);
          resId = resId + 2;
        }
        resDataRetention.children = resChildren;
        resData.push(resDataRetention);

        responseDefer.resolve({ key: 'response', value: resData });
      },
      function (message) {
        console.log('getParams failed: %s', message);

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
      for (var i = 0; i < arguments.length; i++) {
        var key = arguments[i].key;
        var value = arguments[i].value;

        if (key === 'request') {
          reqData = value;
        } else {
          resData = value;
        }
      }

      reqData.sort(function (a, b) {
        return (a.id - b.id);
      });

      resData.sort(function (a, b) {
        return (a.id - b.id);
      });

      var mergedData = reqData.concat(resData);
      dataView.beginUpdate();
      _.each(mergedData, function (item, index) {
        dataView.addItem(item);
      });
      dataView.endUpdate();
    });
  });

  //★ initialize the model
  // dataView = new Slick.Data.DataView({ inlineFilters: true });
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
        dataView.beginUpdate();
        dataView.updateItem(item.id, item);
        dataView.endUpdate();
      }
      e.stopImmediatePropagation();
    }

    // 削除クリックイベントハンドラ
    else if ($target.parent().hasClass('delete-button')
      || $target.hasClass('delete-button')) {
      var items = dataView.getItem(args.row);
      if (items) {
        var parentItem = dataView.getItemById(items.id);
        var childrenId = parentItem.children;
        dataView.beginUpdate();
        _.each(childrenId,
          function (id, index) {
            dataView.deleteItem(id);
          });
        dataView.deleteItem(items.id);
        dataView.endUpdate();
      }
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


