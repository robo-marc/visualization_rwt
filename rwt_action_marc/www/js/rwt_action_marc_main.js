
var dataView = new Slick.Data.DataView({ inlineFilters: true });

$(function () {

  ////////////////////////////////////////
  // variables

  var ros = new ROSLIB.Ros();

  var serviceMap = {};

  var columns = [
    { id: 'tree', name: 'Tree', field: 'tree', width: 170, minWidth: 20, maxWidth: 400, formatter: treeFormatter },
    { id: 'type', name: 'Type', field: 'type', width: 170, minWidth: 20, maxWidth: 400, },
    { id: 'path', name: 'Path', field: 'path', width: 380, minWidth: 20, },
    { id: 'remove', name: '', field: 'remove', width: 30, minWidth: 30, maxWidth: 30, formatter: removeButtonFormatter }
  ];
  var data = [];
  var grid = new Slick.Grid('#myGrid', dataView, columns);

  var actionId = 1;

  ////////////////////////////////////////
  // common

  // initialize screen
  function initScreen() {
    // common
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
        _.each(result.message, function (msg, index) {
          var replace = msg.split('\'').join('\"');
          var obj = JSON.parse(replace);
          objList.push(obj);
        });
        parseObjList(objList);
      },
      function (message) {
        console.log('getActionList failed: %s', message);
      }
    );
  }

  function parseObjList(objList) {
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
    buildPackageSelect(Object.keys(serviceMap));
  }

  function buildPackageSelect(packageList) {
    var optionsHtml = '';

    packageList.sort();
    for (var i = 0; i < packageList.length; i++) {
      optionsHtml += '<option value="' + packageList[i] + '">' + packageList[i] + '</option>';
    }

    var $select = $('#package-select');
    $select.empty();
    $select.append(optionsHtml);
    $select.change();
  }

  $('#package-select').on('change', function () {
    var selectedValue = $('#package-select').val();
    var messageList = serviceMap[selectedValue];
    var optionsHtml = '';

    messageList.sort();
    for (var i = 0; i < messageList.length; i++) {
      optionsHtml += '<option value="' + messageList[i] + '">' + messageList[i] + '</option>';
    }

    var $select = $('#message-select');
    $select.empty();
    $select.append(optionsHtml);
    $select.change();
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
        return spacer + '<span class="toggle expand"></span>' + value;
      } else {
        return spacer + '<span class="toggle collapse"></span>' + value;
      }
    } else {
      return spacer + '<span class="toggle leaf"></span>' + value;
    }
  }

  function removeButtonFormatter(row, cell, value, columnDef, dataContext) {
    if (value === null || value === undefined || dataContext === undefined) { return ''; }
    var parentId = dataContext.parent;
    if (parentId === null) {
      return '<a class="icon delete-button"><i class="material-icons">remove_circle</i></a>';
    }
    return '';
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
    var packageName = $('#package-select').val();
    var messageName = $('#message-select').val();
    var typeName = packageName + '/' + messageName;

    var requestDefer = $.Deferred();
    var promises = [requestDefer.promise()];

    ros.getMessageDetails(typeName,
      function (rosTypeList) { // callback when getMessageDetails() succeeds 
        var dataList = [];
        var excludeTypeList = [];
        var parentInfoList = [];
        var keepParentData = [];

        // push root item
        dataList.push({
          id: actionId,
          indent: 0,
          parent: null,
          tree: 'ActionRoot',
          type: typeName,
          path: typeName,
          remove: '',
          parentPath: String(actionId),
          _collapsed: true,
        });
        parentInfoList.push({
          typeName: typeName,
          parentId: actionId,
          indent: 0,
          path: typeName,
          parentPath: actionId
        });
        actionId++;

        // push child item
        for (var i = 0; i < rosTypeList.length; i++) {

          for (var i2 = 0; i2 < parentInfoList.length; i2++) {
            var parentInfo = parentInfoList[i2];

            for (var i3 = 0; i3 < rosTypeList.length; i3++) {
              var rosType = rosTypeList[i3];

              if (parentInfo.typeName === rosType.type
                && excludeTypeList.indexOf(rosType.type) === -1) {

                var indent = parentInfo.indent + 1;
                var path = parentInfo.parentPath;
                for (var i4 = 0; i4 < rosType.fieldnames.length; i4++) {
                  dataList.push({
                    id: actionId,
                    indent: indent,
                    parent: parentInfo.parentId,
                    tree: rosType.fieldnames[i4],
                    type: rosType.fieldtypes[i4] + (rosType.fieldarraylen[i4] !== -1 ? '[]' : ''),
                    path: parentInfo.path + '/' + rosType.fieldnames[i4],
                    remove: '',
                    parentPath: path + '.' + actionId,
                    _collapsed: true,
                  });
                  keepParentData.push({
                    typeName: rosType.fieldtypes[i4],
                    parentId: actionId,
                    indent: indent,
                    path: parentInfo.path + '/' + rosType.fieldnames[i4],
                    parentPath: path + '.' + actionId
                  });
                  actionId++;
                } // loop end of i4
                excludeTypeList.push(parentInfo.typeName);
              }

            } // loop end of i3
            excludeTypeList.length = 0; // clear array

          } // loop end of i2
          parentInfoList = _.cloneDeep(keepParentData);
          keepParentData.length = 0; // clear array

        } // loop end of i

        requestDefer.resolve({ key: 'ActionRoot', value: dataList });
      },
      function (message) {  // callback when getMessageDetails() fails 
        console.log('getMessageDetails failed: %s', message);
        requestDefer.resolve();
      }
    );

    $.when.apply(null, promises).done(function () {
      // 非同期処理が全部終わったときの処理

      var actionData;
      for (var i = 0; i < arguments.length; i++) { // arguments は forEach() 使えない
        var key = arguments[i].key;
        var value = arguments[i].value;

        if (key === 'ActionRoot') {
          actionData = value; // dataList
        } else {
          actionData = '';
        }
      }

      actionData.sort(function (a, b) {
        if (a.parentPath < b.parentPath) {
          return -1;
        } else if (a.parentPath > b.parentPath) {
          return 1;
        } else {
          return 0;
        }
      });

      dataView.beginUpdate();
      _.each(actionData, function (item, index) {
        dataView.addItem(item);
      });
      dataView.endUpdate();
    });
  });

  function toggleTree(e, args) {
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

  function deleteItems(args) {
    var row = args.row;
    var triggerItem = grid.getDataItem(row);

    if (triggerItem) {
      var triggerItemId = triggerItem.parentPath;
      var deleteTargetIdList = [];
      var allItems = dataView.getItems();

      // search items to delete
      for (var i = allItems.length - 1; i >= 0; i--) {
        var item = allItems[i];
        var rootId = item.parentPath.substring(0, item.parentPath.indexOf('.'));
        if (triggerItemId === rootId) {
          deleteTargetIdList.push(item.id);
        }
      }
      deleteTargetIdList.push(triggerItem.id);

      // delete items
      dataView.beginUpdate();
      for (var j = 0; j < deleteTargetIdList.length; j++) {
        dataView.deleteItem(deleteTargetIdList[j]);
      }
      dataView.endUpdate();
    }
  }

  grid.onClick.subscribe(function gridClickhandler(e, args) {
    var $target = $(e.target);

    // event handler: toggle tree button
    if ($target.hasClass('toggle')) {
      toggleTree(e, args);
    }

    // event handler: click remove button
    else if ($target.parent().hasClass('delete-button')
      || $target.hasClass('delete-button')) {
      deleteItems(args);
    }
  });

  dataView.onRowCountChanged.subscribe(function (e, args) {
    grid.updateRowCount();
    grid.render();
    grid.resizeCanvas();
    grid.autosizeColumns();
  });

  dataView.onRowsChanged.subscribe(function (e, args) {
    grid.invalidateRows(args.rows);
    grid.render();
  });

  $(window).on('load resize', function () {
    grid.resizeCanvas();
    grid.autosizeColumns();

    // prevent the delete button from being hidden when switching screens vertically
    grid.resizeCanvas();
  });


  ////////////////////////////////////////
  // start screen
  initScreen();
});
