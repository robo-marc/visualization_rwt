////////////////////////////////////////
// global variables

var dataView = new Slick.Data.DataView({ inlineFilters: true });

function getTargetTopicTypeList() {
  return undefined;
}


$(function () {

  ////////////////////////////////////////
  // variables

  var ros = new ROSLIB.Ros();

  // config
  var refleshRate = 1;

  // status
  var subscribingMap = {};
  var subscribingValueMap = {};
  var targetTopicTypeList;
  var isFirstAdjusted = false; // adjust column width

  // for grid
  var checkboxSelector = new Slick.CheckboxSelectColumn({
    cssClass: 'slick-cell-checkboxsel',
    hideInColumnTitleRow: true,
  });

  var checkboxColumnDef = checkboxSelector.getColumnDefinition();
  checkboxColumnDef.formatter = checkboxCustomFormatter;

  var columns = [
    checkboxColumnDef,
    { id: 'topic', name: 'Topic', field: 'topic', width: 200, formatter: treeFormatter },
    { id: 'type', name: 'Type', field: 'type', width: 260 },
    { id: 'bandwidth', name: 'Bandwidth', field: 'bandwidth', width: 100, maxWidth: 100 },
    { id: 'hz', name: 'Hz', field: 'hz', width: 100, maxWidth: 100 },
    { id: 'value', name: 'Value', field: 'value', width: 260 },
  ];
  var data = [];
  var grid = new Slick.Grid('#myGrid', dataView, columns);


  ////////////////////////////////////////
  // functions

  // initialize screen
  function initScreen() {
    // common
    ros.autoConnect();

    // If not an iframe, parent equals to myself
    targetTopicTypeList = window.parent.getTargetTopicTypeList();

    // for grid
    dataView.beginUpdate();
    dataView.setItems(data);
    dataView.setFilter(myFilter);
    dataView.endUpdate();

    // start
    setInterval(refreshTopics, 1000 / refleshRate);
    // refreshTopics(); // for debug
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
    var spacer = RwtUtils.getTreeSpacer(dataContext['indent']);
    var idx = dataView.getIdxById(dataContext.id);
    if (data[idx + 1] && data[idx + 1].indent > data[idx].indent) {
      if (dataContext._collapsed) {
        return spacer + RwtUtils.getTreeExpandButton() + value;
      } else {
        return spacer + RwtUtils.getTreeCollapseButton() + value;
      }
    } else {
      return spacer + RwtUtils.getTreeLeafButton() + value;
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
        defer.resolve(result.info);
      },
      function (message) {
        console.log('getMonitoringInfo failed: ' + message);
        defer.resolve();
      }
    );
    return defer.promise();
  }

  function decodeTypeDefs(type_defs, root) {
    // It calls itself recursively to resolve type definition
    // using hint_defs.
    var decodeTypeDefsRec = function (theType, theValue, parent, hint_defs) {
      var typeList = [];
      var fieldValue;
      var subType;
      var subTypeTmpResult;
      var arrValue;

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
          _collapsed: true,
        };
        typeList.push(item);

        // get value
        fieldValue = undefined;
        if (theValue !== undefined) {
          fieldValue = theValue[fieldName];
        }

        // get subtype
        subType = undefined;
        for (var j = 0; j < hint_defs.length; j++) {
          if (hint_defs[j].type.toString() === fieldType.toString()) {
            subType = hint_defs[j];
            break;
          }
        }

        if (subType) {
          if (isArray) {
            // object array
            if (_.isArray(fieldValue) && fieldValue.length > 0) {
              for (var valueIndex = 0; valueIndex < fieldValue.length; valueIndex++) {
                arrValue = fieldValue[valueIndex];
                subTypeTmpResult = decodeTypeDefsRec(subType, arrValue, item, hint_defs);

                // build subtype parent like '[0]'
                var indexName = '[' + valueIndex + ']';
                var arrItemParent = _.cloneDeep(item);
                arrItemParent.id = item.id + '.' + indexName;
                arrItemParent.topic = indexName;
                arrItemParent.type = fieldType;
                arrItemParent.parent = item.id;
                arrItemParent.indent = item.indent + 1;
                typeList.push(arrItemParent);

                for (var subTypeIndex = 0; subTypeIndex < subTypeTmpResult.length; subTypeIndex++) {
                  var arrItem = _.cloneDeep(subTypeTmpResult[subTypeIndex]);

                  var tmpId = arrItem.id;
                  var relativePathFromParent = tmpId.replace(arrItemParent.parent, '');

                  arrItem.id = arrItemParent.id + relativePathFromParent;
                  arrItem.parent = arrItem.id.substring(0, arrItem.id.lastIndexOf('.'));
                  arrItem.indent = arrItem.indent + 1;

                  typeList.push(arrItem);
                }
              }
            } else {
              // cannot build subtype tree because no value of object array
            }
          } else {
            // simple object, not array
            subTypeTmpResult = decodeTypeDefsRec(subType, fieldValue, item, hint_defs);
            typeList = typeList.concat(subTypeTmpResult);
          }
        } else {
          // primitive type

          // format value and set
          if (isArray && _.isArray(fieldValue)) {
            if ((fieldType === 'string' || fieldType === 'char')) {
              if (fieldValue.length <= 0) {
                item.value = '[]';
              } else {
                item.value = '[\'' + fieldValue.join('\', \'') + '\']';
              }
            } else {
              item.value = '(' + fieldValue.join(', ') + ')';
            }
          } else {
            if ((fieldType === 'string' || fieldType === 'char') && fieldValue) {
              fieldValue = '\'' + fieldValue + '\'';
            }
            item.value = fieldValue;
          }
        }
      }

      return typeList;
    }; // end of decodeTypeDefsRec

    var value = subscribingValueMap[root.topic];
    return decodeTypeDefsRec(type_defs[0], value, root, type_defs);
  }

  function getMonitoringInfoByName(list, name) {
    if (!list) {
      return undefined;
    }
    for (var i = 0; i < list.length; i++) {
      if (list[i].name === name) {
        return list[i];
      }
    }
    return undefined;
  }

  function refreshTopics() {
    ros.getTopics(function (topicInfo) {
      var monitoringInfoDefer = $.Deferred();
      var messageDetailPromises = [];
      var topicDetailMap = {}; // key: topic name, value: message detail

      if (targetTopicTypeList !== undefined) {
        // moveit only
        var comparisonItem = _.cloneDeep(targetTopicTypeList);
        var keepInfo = { topics: [], types: [] };

        for (var i = 0; i < topicInfo.topics.length; i++) {
          if (comparisonItem.indexOf(topicInfo.types[i]) >= 0) {
            keepInfo.topics.push(topicInfo.topics[i]);
            keepInfo.types.push(topicInfo.types[i]);
          }
        }
        topicInfo = _.cloneDeep(keepInfo);
      }
      // topic and moveit common
      _.each(topicInfo.topics, function (topicName, index) {
        var topicType = topicInfo.types[index];

        var promise = getMessageDetailsAsync(
          topicType,
          function (result) {
            topicDetailMap[topicName] = {
              topicType: topicType,
              detail: result
            };
          }
        );
        messageDetailPromises.push(promise);
      });

      $.when.apply(null, messageDetailPromises).done(function () {
        if (Object.keys(subscribingMap).length > 0) {
          // get list of bandwidth, hz
          var promise = getMonitoringInfoAsync();
          promise.done(function (monitoringInfoList) {
            monitoringInfoDefer.resolve(monitoringInfoList);
          });
        } else {
          monitoringInfoDefer.resolve();
        }
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

        if (!isFirstAdjusted) {
          adjustColumnWidth();
          isFirstAdjusted = true;
        }
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
        });

      } else {
        item._checked = false;

        if (item.topic in subscribingMap) {
          //unsubscribe
          subscribingMap[item.topic].unsubscribe();
          delete subscribingValueMap[item.topic];
          delete subscribingMap[item.topic];
        }
        // stop monitoring of bandwidth, Hz
        stopMonitoringAsync(item.topic, item.type);
      }
    }
    e.stopImmediatePropagation();
  }

  grid.onClick.subscribe(function gridClickhandler(e, args) {
    var $target = $(e.target);

    if (RwtUtils.isTreeToggleButton($target)) {
      toggleTree(e, args);
    } else if ($target.hasClass('checkbox')) {
      toggleCheckbox(e, args);
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

  $(window).on('load resize', function () {
    adjustColumnWidth();
  });

  function adjustColumnWidth() {
    grid.resizeCanvas();
    grid.autosizeColumns();
  }


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
