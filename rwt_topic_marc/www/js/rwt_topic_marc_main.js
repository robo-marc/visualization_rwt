var dataView = new Slick.Data.DataView({ inlineFilters: true });

$(function () {

  var ros = new ROSLIB.Ros();

  var topicTypeDetail = {};
  var info = {};
  var topicId = 0;

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
    { id: 'hz', name: 'Hz', field: 'hz', width: 50, minWidth: 20, maxWidth: 50 },
    { id: 'value', name: 'Value', field: 'value', width: 260, minWidth: 20, maxWidth: 900 },
  ];
  var data = [];
  var grid = new Slick.Grid('#myGrid', dataView, columns);

  var topicDetailMap = {}; // key: topic name, value: message detail

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

    // draw grid
    setInterval(getTopics, 1000);
    // getTopics();
  }


  function checkboxCustomFormatter(row, cell, value, columnDef, dataContext) {
    var idx = dataView.getIdxById(dataContext.id);
    if (data[idx] && data[idx].parent === null && !(dataContext._checked)) {
      // return checkboxDefaultFormatter(row, cell, value, columnDef, dataContext);
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

        topicId++;
        var newId = parent.id + '.' + fieldName;
        var item = {
          id: newId,
          topic: fieldName,
          type: fieldType + ((arrayLen === -1) ? '' : '[]'),
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
          typeList = typeList.concat(sub_type_result);
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
              item.value = '(' + valueObj.join(', ') + ')';
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

  function getTopics() {
    ros.getTopics(function (topicInfo) {
      // console.log(topicInfo);

      var promises = [];

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
          },
          function (mes) {
            console.log(mes);
          }
        );
        promises.push(promise);
      });

      // TODO: bandwidth, hz を取得する

      $.when.apply(null, promises).done(function () {
        // format type information
        var fieldList = [];

        _.each(topicDetailMap, function (details, topicName) {
          topicId++;
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
          // Inherit tree OPEN / CLOSE and check ON / OFF status from previous data
          for (var i = 0; i < fieldList.length; i++) {
            var newItem = fieldList[i];
            var oldItem = dataView.getItemById(newItem.id);
            if (oldItem) {
              newItem._collapsed = oldItem._collapsed;
              newItem._checked = oldItem._checked;
              if (newItem.parent === null && newItem._checked) {
                newItem.value = undefined; // hide 'not monitored'
              }
            } else {
              // Do nothing because old items are not found
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

  // get Hz Bw
  function getHzBw(topic, type, info) {
    var defer = $.Deferred();

    ros.getTopicInfo(topic, type, function (value) {
      console.log('---- Hz Bw ----');
      // console.log(topic + '  /  ' + type);
      // console.log(value);
      info = value;
      defer.resolve();
    });
    return defer.promise();
  }

  function subscribeTopic(topic, type, subTopic) {
    var defer = $.Deferred();

    var sub = new ROSLIB.Topic({
      ros: ros,
      name: topic,
      messageType: type
    });
    sub.subscribe(function (message) {
      console.log('---- Received message ----');
      // console.log(topic + '  /  ' + type);
      // console.log(message);
      // console.log(message.data);

      defer.resolve();
    });

    return defer.resolve();
  }

  function getMessageDetailsAsync(message, callback, failedCallback) {
    var defer = $.Deferred();

    ros.getMessageDetails(message,
      function (result) {
        // console.log('getMessageDetailsAsync success');
        callback(result);
        defer.resolve();
      },
      function (message) {
        console.log('getMessageDetailsAsync failed');
        failedCallback(message);
        defer.resolve();
      }
    );

    return defer.promise();
  }

  // var isCheckbox = false;
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

  function changeCheckbox(e, args) {
    var item = dataView.getItem(args.row);
    var sub;
    if (item) {
      if (!item._checked) {
        item._checked = true;

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
      }
    }
    e.stopImmediatePropagation();
  }

  grid.onClick.subscribe(function gridClickhandler(e, args) {
    var $target = $(e.target);

    // event handler: toggle tree button
    if ($target.hasClass('toggle')) {
      toggleTree(e, args);
    }

    if ($target.hasClass('checkbox')) {
      changeCheckbox(e, args);
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
