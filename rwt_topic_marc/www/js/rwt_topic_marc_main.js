////////////////////////////////////////
// global variables

var dataView = new Slick.Data.DataView({ inlineFilters: true });


$(function () {

  ////////////////////////////////////////
  // variables

  var ros = new ROSLIB.Ros();

  var subscribingMap = {};
  var subscribingValueMap = {};

  var checkboxSelector = new Slick.CheckboxSelectColumn({
    cssClass: 'slick-cell-checkboxsel',
    hideInColumnTitleRow: true,
  });

  var checkboxColumnDef = checkboxSelector.getColumnDefinition();
  checkboxColumnDef.formatter = checkboxCustomFormatter;

  var columns = [
    checkboxColumnDef,
    { id: 'topic', name: 'Topic', field: 'topic', width: 200, minWidth: 20, maxWidth: 300, formatter: treeFormatter },
    { id: 'type', name: 'Type', field: 'type', width: 260, minWidth: 20, maxWidth: 900 },
    { id: 'bandwidth', name: 'Bandwidth', field: 'bandwidth', width: 100, minWidth: 20, maxWidth: 100 },
    { id: 'hz', name: 'Hz', field: 'hz', width: 100, minWidth: 20, maxWidth: 100 },
    { id: 'value', name: 'Value', field: 'value', width: 260, minWidth: 20, maxWidth: 900 },
  ];
  var data = [];
  var grid = new Slick.Grid('#myGrid', dataView, columns);

  var topicDetailMap = {}; // key: topic name, value: message detail


  ////////////////////////////////////////
  // functions

  // initialize screen
  function initScreen() {
    // common
    ros.autoConnect();

    // for grid
    dataView.beginUpdate();
    dataView.setItems(data);
    dataView.setFilter(myFilter);
    dataView.endUpdate();

    // start
    setInterval(refreshTopics, 1000);
    // renderTopics();
  }


  function checkboxCustomFormatter(row, cell, value, columnDef, dataContext) {
    var idx = dataView.getIdxById(dataContext.id);
    if (data[idx] && data[idx].parent === null && !(dataContext._checked)) {
      return '<input type="checkbox" class="checkbox">';
    } else if (data[idx] && data[idx].parent === null) {
      return '<input type="checkbox" class="checkbox" checked="checked">';
    }

    return '';
  }

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

  function getMessageDetailsAsync(message, callback) {
    var defer = $.Deferred();

    ros.getMessageDetails(message,
      function (result) {
        // console.log('getMessageDetailsAsync success');
        callback(result);
        defer.resolve();
      },
      function (message) {
        console.log('getMessageDetailsAsync failed: ' + message);
        defer.resolve();
      }
    );

    return defer.promise();
  }

  function startMonitoringAsync(topicName, topicType) {
    var defer = $.Deferred();

    ros.startMonitoring(topicName, topicType,
      function (result) {
        // console.log('startMonitoring success: ' + topicName);
        defer.resolve();
      },
      function (message) {
        console.log('startMonitoring failed: ' + topicName, ': ' + message);
        defer.resolve();
      }
    );
    return defer.promise();
  }

  function stopMonitoringAsync(topicName, topicType) {
    var defer = $.Deferred();

    ros.stopMonitoring(topicName, topicType,
      function (result) {
        // console.log('stopMonitoring success: ' + topicName);
        defer.resolve();
      },
      function (message) {
        console.log('stopMonitoring failed: ' + topicName, ': ' + message);
        defer.resolve();
      }
    );
    return defer.promise();
  }

  function getMonitoringInfoAsync() {
    var defer = $.Deferred();

    ros.getMonitoringInfo(
      function (result) {
        // console.log('getMonitoringInfo success');
        defer.resolve(result.info);
      },
      function (message) {
        console.log('getMonitoringInfo failed: ' + message);
        defer.resolve();
      }
    );
    return defer.promise();
  }

  function getValueFromObj(name, valueObj, path) {
    var keys = Object.keys(valueObj);
    var value;
    for (var i = 0; i < keys.length; i++) {
      var key = keys[i];
      value = valueObj[keys[i]];
      if (keys[i] === name) {
        if (typeof value === 'object') {
          value = undefined; // hide value of object 
        }
        path = path + '.' + key;
        break;
      } else {
        if (typeof value === 'object') {
          var result = getValueFromObj(name, value, path);
          if (result.path !== '') {
            path = '.' + key + result.path;
            value = result.value;
            break;
          }
        } else {
          // name missmatch
          value = undefined;
        }
      }
    }
    return { path: path, value: value };
  }

  function decodeTypeDefs(type_defs, root) {
    // It calls itself recursively to resolve type definition
    // using hint_defs.
    var decodeTypeDefsRec = function (theType, parent, hint_defs) {
      var typeList = [];
      var sub_type;

      for (var i = 0; i < theType.fieldnames.length; i++) {
        var arrayLen = theType.fieldarraylen[i];
        var fieldName = theType.fieldnames[i];
        var fieldType = theType.fieldtypes[i];

        var newId = parent.id + '.' + fieldName;
        var isArray = (arrayLen !== -1);
        var item = {
          id: newId,
          topic: fieldName,
          type: fieldType + (isArray ? '[]' : ''),
          bandwidth: undefined,
          hz: undefined,
          value: undefined,
          parent: parent.id,
          indent: parent.indent + 1,
          root: root.topic,
          subType: true,
          _collapsed: true,
        };
        typeList.push(item);

        // lookup the name
        sub_type = undefined;
        for (var j = 0; j < hint_defs.length; j++) {
          if (hint_defs[j].type.toString() === fieldType.toString()) {
            sub_type = hint_defs[j];
            break;
          }
        }
        if (sub_type) {
          var sub_type_result = decodeTypeDefsRec(sub_type, item, hint_defs);
          if (isArray) {
            // get object array value
            var valueObj2 = subscribingValueMap[root.topic];
            if (valueObj2) {
              var itemPropNames2 = item.id.split('.');

              // start search from 1 because itemPropNames2[0] is topicName
              for (var k2 = 1; k2 < itemPropNames2.length; k2++) {
                var propName2 = itemPropNames2[k2];
                if (propName2 in valueObj2) {
                  valueObj2 = valueObj2[propName2];
                }
              }
              for (var valueObjIndex = 0; valueObjIndex < valueObj2.length; valueObjIndex++) {
                var indexName = '[' + valueObjIndex + ']';
                var arrItemParent = _.cloneDeep(item);
                arrItemParent.id = item.id + '.' + indexName;
                arrItemParent.topic = indexName;
                arrItemParent.type = fieldType;
                arrItemParent.parent = item.id;
                arrItemParent.indent = item.indent + 1;
                typeList.push(arrItemParent);

                for (var subTypeIndex = 0; subTypeIndex < sub_type_result.length; subTypeIndex++) {
                  var arrItem = _.cloneDeep(sub_type_result[subTypeIndex]);
                  var result = getValueFromObj(arrItem.topic, valueObj2[valueObjIndex], '');

                  arrItem.id = arrItemParent.id + result.path;
                  arrItem.parent = arrItem.id.substring(0, arrItem.id.lastIndexOf('.'));
                  arrItem.indent = arrItem.indent + 1;
                  arrItem.value = result.value;

                  // console.log(arrItem);
                  typeList.push(arrItem);
                }
              }
            }
          } else {
            typeList = typeList.concat(sub_type_result);
          }
        } else {
          item.subType = false;

          // get value
          var valueObj = subscribingValueMap[root.topic];
          if (valueObj) {
            var itemPropNames = item.id.split('.');

            // start search from 1 because itemPropNames[0] is topicName
            for (var k = 1; k < itemPropNames.length; k++) {
              var propName = itemPropNames[k];
              if (propName in valueObj) {
                valueObj = valueObj[propName];
              }
            }

            if (_.isArray(valueObj) && valueObj.length > 0) {
              if (typeof valueObj[0] === 'object') {
                // object array valueis already set
              } else {
                item.value = '(' + valueObj.join(', ') + ')';
              }
            } else {
              item.value = valueObj;
            }
          }
        }
      }

      return typeList;
    }; // end of decodeTypeDefsRec

    return decodeTypeDefsRec(type_defs[0], root, type_defs);
  }

  function getMonitoringInfoByName(list, name) {
    for (var i = 0; i < list.length; i++) {
      if (list[i].name === name) {
        return list[i];
      }
    }
    return undefined;
  }

  function refreshTopics() {
    ros.getTopics(function (topicInfo) {
      // console.log(topicInfo);

      var monitoringInfoDefer = $.Deferred();
      var messageDetailPromises = [];

      _.each(topicInfo.topics, function (topicName, index) {
        var topicType = topicInfo.types[index];

        var promise = getMessageDetailsAsync(
          topicType,
          function (result) {
            // console.log(result);
            topicDetailMap[topicName] = {
              topicType: topicType,
              detail: result
            };
          }
        );
        messageDetailPromises.push(promise);
      });

      $.when.apply(null, messageDetailPromises).done(function () {
        // get list of bandwidth, hz
        var promise = getMonitoringInfoAsync();
        promise.done(function (monitoringInfoList) {
          monitoringInfoDefer.resolve(monitoringInfoList);
        });
      });

      monitoringInfoDefer.promise().done(function (monitoringInfoList) {
        // format type information
        var fieldList = [];

        _.each(topicDetailMap, function (details, topicName) {
          var parent = {
            id: topicName,
            topic: topicName,
            type: details.topicType,
            bandwidth: undefined,
            hz: undefined,
            value: 'not monitored',
            parent: null,
            indent: 0,
            root: topicName,
            subType: true,
            _collapsed: true,
            _checked: false,
          };

          fieldList.push(parent);

          var decoded = decodeTypeDefs(details.detail, parent);
          fieldList = fieldList.concat(decoded);

        });

        if (data.length > 0) {
          var isRootCheckOff = false;
          for (var i = 0; i < fieldList.length; i++) {
            // Inherit data status from previous data
            // (tree OPEN/CLOSE, check ON/OFF, value )
            var newItem = fieldList[i];
            var oldItem = dataView.getItemById(newItem.id);
            if (oldItem) {
              newItem._collapsed = oldItem._collapsed;
              newItem._checked = oldItem._checked;
              if (newItem.parent === null) {
                isRootCheckOff = !newItem._checked;
              }
              if (newItem.parent === null && newItem._checked) {
                newItem.value = undefined; // hide 'not monitored'
              } else if (newItem.parent !== null && isRootCheckOff) {
                newItem.value = oldItem.value; // keep last value
              }
            } else {
              // Do nothing because old items are not found
            }

            // get bandwidth, hz
            if (newItem.parent === null && newItem._checked) {
              var info = getMonitoringInfoByName(monitoringInfoList, newItem.topic);
              if (info) {
                newItem.bandwidth = info.bw;
                newItem.hz = info.hz;
              }
            }
          }
        }

        fieldList.sort(function (a, b) {
          if (a.root < b.root) {
            return -1;
          } else if (a.root > b.root) {
            return 1;
          }
          return 0;
        });

        data = fieldList;

        // draw grid
        dataView.setItems(fieldList);
        grid.invalidate();
      });
    });
  }


  ////////////////////////////////////////
  // grid events

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

  function toggleCheckbox(e, args) {
    var item = dataView.getItem(args.row);
    var sub;
    if (item) {
      if (!item._checked) {
        item._checked = true;

        // start monitoring of bandwidth, Hz
        startMonitoringAsync(item.topic, item.type);

        //subscribe
        sub = new ROSLIB.Topic({
          ros: ros,
          name: item.topic,
          messageType: item.type
        });
        subscribingMap[item.topic] = sub;
        sub.subscribe(function (msg) {
          subscribingValueMap[item.topic] = msg;
          // console.log(subscribingValueMap);
        });

      } else {
        item._checked = false;

        //unsubscribe
        subscribingMap[item.topic].unsubscribe();
        delete subscribingValueMap[item.topic];
        delete subscribingMap[item.topic];

        // stop monitoring of bandwidth, Hz
        stopMonitoringAsync(item.topic, item.type);
      }
    }
    e.stopImmediatePropagation();
  }

  grid.onClick.subscribe(function gridClickhandler(e, args) {
    var $target = $(e.target);

    if ($target.hasClass('toggle')) {
      toggleTree(e, args);
    } else if ($target.hasClass('checkbox')) {
      toggleCheckbox(e, args);
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


  ////////////////////////////////////////
  // resize events

  $(window).on('load resize', function () {
    grid.resizeCanvas();
    grid.autosizeColumns();

    // prevent the delete button from being hidden when switching screens vertically
    grid.resizeCanvas();
  });


  ////////////////////////////////////////
  // button events

  function closeAllTree(toClosed) {
    dataView.beginUpdate();
    var items = dataView.getItems();
    for (var i = 0; i < items.length; i++) {
      items[i]._collapsed = toClosed;
    }
    dataView.setItems(items);
    dataView.endUpdate();
    grid.invalidate();
  }

  $('#expand-button').on('click', function (e) {
    e.preventDefault();
    closeAllTree(false);
  });

  $('#collapse-button').on('click', function (e) {
    e.preventDefault();
    closeAllTree(true);
  });


  ////////////////////////////////////////
  // start screen
  initScreen();
});
