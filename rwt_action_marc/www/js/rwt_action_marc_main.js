
var dataView = new Slick.Data.DataView({ inlineFilters: true });

$(function () {

  ////////////////////////////////////////
  // variables

  var ros = new ROSLIB.Ros();

  var serviceMap = new Map();

  var columns = [
    { id: 'tree', name: 'Tree', field: 'tree', width: 200, minWidth: 20, maxWidth: 300, formatter: treeFormatter },
    { id: 'type', name: 'Type', field: 'type', width: 260, minWidth: 20, maxWidth: 900, },
    { id: 'path', name: 'Path', field: 'path', width: 260, minWidth: 20, maxWidth: 300, },
    { id: 'remove', name: 'Remove', field: 'remove', width: 100, minWidth: 20, maxWidth: 200, formatter: removeButtonFormatter }
  ];
  var data = [];
  var grid = new Slick.Grid('#myGrid', dataView, columns);

  var actionId = 1;
  var deleteItem = [];

  ////////////////////////////////////////
  // common

  // initialize screen
  function initScreen() {

    ros.autoConnect();

    // for grid
    dataView.beginUpdate();
    dataView.setItems(data);
    dataView.setFilter(myFilter);
    dataView.endUpdate();

    // for dropdown
    getActionList();
  }


  ////////////////////////////////////////
  // dropdown

  function getActionList() {
    ros.getActionList(
      function (result) {
        var objList = [];
        _.each(result.message,
          function (msg, index) {
            var replace = msg.split('\'').join('\"');
            var obj = JSON.parse(replace);
            objList.push(obj);
          });
        // console.log(objList);
        getActionDetail(objList);
      },
      function (mes) {
        console.log(mes);
      }
    );
  }

  // actionのリストを表示するリストボックス
  function getActionDetail(objList) {
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
          $('#package-select').append('<option value="' + serviceStr + '">' + serviceStr + '</option>');
        }
      }
    }
    $('#package-select').change();
    // ////// 不要？
    // var messageList = serviceMap.get(firstAddServiceStr);
    // for (var k = 0; k < messageList.length; k++) {
    //   $('#message-select').append('<option value="' + messageList[k] + '">' + messageList[k] + '</option>');
    // }
  }

  $('#package-select').on('change', function () {
    var selectedValue = $('#package-select').val();
    $('#message-select').empty();
    var messageList = serviceMap.get(selectedValue);
    for (var k = 0; k < messageList.length; k++) {
      $('#message-select').append('<option value="' + messageList[k] + '">' + messageList[k] + '</option>');
    }
  });


  ////////////////////////////////////////
  // grid

  function treeFormatter(row, cell, value, columnDef, dataContext) {
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
  }

  function removeButtonFormatter(row, cell, value, columnDef, dataContext) {
    // if (value === null || value === undefined || dataContext === undefined) { return ''; }
    // var parentId = dataContext.parent;
    // if (parentId === null) {
    return '<button class="delete-button"><span class="glyphicon glyphicon-minus-sign" aria-hidden="true"></span></button>';
    // }
    // return '';
  }

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

  $('#add-button').on('click', function () {
    var parents = [];
    var serviceName = $('#package-select').val();
    var messageName = $('#message-select').val();
    var typeName = serviceName + '/' + messageName;

    var requestDefer = $.Deferred();
    var promises = [requestDefer.promise()];

    ros.getMessageDetails(typeName,
      function (types) {

        var actionData = [];
        var ExcludItem = [];
        var parentData = [];
        var keepParentData = [];
        actionData.push({
          id: actionId,
          indent: 0,
          parent: null,
          tree: 'ActionRoot',
          type: typeName,
          path: typeName,
          remove: '',
          parentPath: actionId
        });

        parentData.push({ typeName: typeName, parentId: actionId, indent: 0, parentPath: actionId });
        actionId++;

        for (var i = 0; i < types.length; i++) {
          _.each(parentData, function (data, index) {
            _.each(types, function (item, index) {
              // console.log(data);
              if (data.typeName === item.type && ExcludItem.indexOf(item.type) === -1) {
                var indent = data.indent + 1;
                var path = data.parentPath;
                for (var j = 0; j < item.fieldnames.length; j++) {
                  actionData.push({
                    id: actionId,
                    indent: indent,
                    parent: data.parentId,
                    tree: item.fieldnames[j],
                    type: item.fieldtypes[j] + (item.fieldarraylen[j] !== -1 ? '[]' : ''),
                    path: item.fieldtypes[j],
                    remove: '',
                    parentPath: path + '.' + actionId,
                  });
                  keepParentData.push({ typeName: item.fieldtypes[j], parentId: actionId, indent: indent, parentPath: path + '.' + actionId });

                  //TODO: 孫の代までのデータ追加
                  deleteItem.push({ parentId: data.parentId, childId: actionId });


                  actionId++;
                  // console.log(actionData);
                }
                ExcludItem.push(data.typeName);
              }
            });
            ExcludItem.length = 0;
          });
          parentData = _.cloneDeep(keepParentData);
          keepParentData.length = 0;

        }
        requestDefer.resolve({ key: 'ActionRoot', value: actionData });
      },
      function (message) {
        console.log('getMessageDetails failed: %s', message);
        requestDefer.resolve();
      }
    );

    $.when.apply(null, promises).done(function () {
      // 非同期処理が全部終わったときの処理

      var actionData;
      var resData;
      for (var i = 0; i < arguments.length; i++) { // arguments は forEach() 使えない
        var key = arguments[i].key;
        var value = arguments[i].value;
        console.log(value);

        if (key === 'ActionRoot') {
          actionData = value;
        } else {
          actionData = '';
        }
      }

      for (var j = 0; j < actionData.length - 1; j++) {
        for (var k = actionData.length - 1; k > j; k--) {
          if (actionData[k].parentPath < actionData[k - 1].parentPath) {
            var tmp = actionData[k];
            actionData[k] = actionData[k - 1];
            actionData[k - 1] = tmp;
          }
        }
      }

      console.log(actionData);
      dataView.beginUpdate();
      _.each(actionData, function (item, index) {
        dataView.addItem(item);
      });
      dataView.endUpdate();
    });
  });

  grid.onClick.subscribe(function gridClickhandler(e, args) {
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
    //TODO: 削除処理のアップデート
    else if ($target.parent().hasClass('delete-button')
      || $target.hasClass('delete-button')) {
      var row = args.row;
      var items = grid.getDataItem(row);

      // var parentItem = dataView.getItemById(items.id);
      // var parent = parentItem.parentPath;
      // console.log(parent);
      // var childrenId = parent.split('.');
      console.log(deleteItem);
      // console.log(item.childId);
      if (items) {
        // dataView.beginUpdate();
        _.each(deleteItem, function (item, index) {
          if (item.parentId === items.id) {
            dataView.deleteItem(item.childId);
          }
        });
        dataView.deleteItem(items.id);
        grid.invalidateAllRows();
        grid.render();
        // dataView.endUpdate();
      }
      // data.splice(args.row, 1);
      // grid.invalidate();
    }
  });

  dataView.onRowCountChanged.subscribe(function (e, args) {
    grid.updateRowCount();
    grid.render();
  });

  dataView.onRowsChanged.subscribe(function (e, args) {
    grid.invalidateRows(args.rows);
    grid.render();
  });


  ////////////////////////////////////////
  // start screen
  initScreen();
});
