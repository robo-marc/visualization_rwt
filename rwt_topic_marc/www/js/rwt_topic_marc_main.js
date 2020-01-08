var dataView = new Slick.Data.DataView({ inlineFilters: true });

$(function () {

  var ros = new ROSLIB.Ros();

  var checkboxSelector;

  var topicTypeDetail = {};
  var info = {};
  var topicId = 0;
  checkboxSelector = new Slick.CheckboxSelectColumn({
    cssClass: 'slick-cell-checkboxsel'
  });

  // ★グリッドアイテム取得/////////////////////////////////////////
  var columns = [
    checkboxSelector.getColumnDefinition(),
    { id: 'topic', name: 'Topic', field: 'topic', width: 200, minWidth: 20, maxWidth: 300, sortable: true, formatter: treeFormatter },
    { id: 'type', name: 'Type', field: 'type', width: 260, minWidth: 20, maxWidth: 900, sortable: true },
    { id: 'bandwidth', name: 'Bandwidth', field: 'bandwidth', width: 50, minWidth: 20, maxWidth: 300, sortable: true },
    { id: 'hz', name: 'Hz', field: 'hz', width: 50, minWidth: 20, maxWidth: 300, sortable: true },
    { id: 'value', name: 'Value', field: 'value', width: 100, minWidth: 20, maxWidth: 300, sortable: true },
  ];
  var data = [];
  var grid = new Slick.Grid('#myGrid', dataView, columns);

  var parentTopicDetailList = [];
  var allTopicDetailList = [];
  var detailItemList = [];
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
    getTopics();
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

  function getDetailList(topic, type) {

    //get rootTopic
    console.log(topicId + ':' + parentTopicDetailList.length);
    // if (parentTopicDetailList.length === 0) {
    //   parentTopicDetailList.push({
    //     id: topicId,
    //     indent: 0,
    //     parent: null,
    //     topic: topic,
    //     type: type,
    //     remove: '',
    //     parentPath: String(topicId),
    //     _collapsed: true,
    //   });
    //   topicId++;
    //   return getDetailList(topic, type);
    // }


    var promise = getMessageDetailsAsync(type,
      function (result) {
        // detailItemList.push(result);
        topicDetailMap[topic] = result;
      },
      function (mes) {
        console.log(mes);
      }
    );

    // $.when(promise).done(function () {
    // console.log("aaaa");
    var indent = 1;
    if (detailItemList.length > 1) {
      // console.log("bbbb");
      // console.log(detailItemList);
      _.each(detailItemList, function (item, index) {
        // console.log(item);
        // console.log(type);
        _.each(item, function (item2, index) {

          if (item2.type === type) {
            console.log(item2);
            for (var i = 0; i < item2.fieldnames.length; i++) {
              parentTopicDetailList.push({
                id: topicId,
                indent: indent,
                parent: topic,
                topic: item2.fieldnames[i],
                type: item2.fieldtypes[i],
                remove: '',
                parentPath: String(topicId),
                _collapsed: true,
              });
              topicId++;
            }
            // return item2.fieldnames[i], item2.fieldtypes[i];
          }
          indent++;
        });
        // return getDetailList(item,)
      });
    }
    allTopicDetailList.push(parentTopicDetailList);

    console.log(allTopicDetailList);

    // last processing
    return;
    // });
  }


  function getFieldList(typeInfo, parent) {
    var typeList = [];

    _.each(typeInfo, function (fieldType, fieldName) {
      topicId++;
      var item = {
        id: topicId,
        topic: fieldName,
        type: fieldType + (_.isArray(fieldType) ? '[]' : ''),
        bandwidth: undefined,
        hz: undefined,
        value: undefined,
        parent: parent.id,
        indent: parent.indent + 1,
        _collapsed: true,
      };
      typeList.push(item);

      if (typeof fieldType === 'object' && !(_.isArray(fieldType))) {
        // console.log('[name, type]: [' + fieldName + ', ' + fieldType + '] is object');
        // typeList.push(fieldName); // For set all member, but not implemented.
        var children = getFieldList(fieldType, item);
        _.each(children, function (child, index) {
          // var s = fieldName + '/' + child;
          typeList.push(child);
        });

        // } else {
        //   typeList.push(item);
      }
    });

    return typeList;
  }


  var decodeTypeDefs = function (defs) {
    // var that = this;

    // calls itself recursively to resolve type definition using hints.
    var decodeTypeDefsRec = function (theType, hints) {
      var typeDefDict = {};
      for (var i = 0; i < theType.fieldnames.length; i++) {
        var arrayLen = theType.fieldarraylen[i];
        var fieldName = theType.fieldnames[i];
        var fieldType = theType.fieldtypes[i];
        if (fieldType.indexOf('/') === -1) { // check the fieldType includes '/' or not
          if (arrayLen === -1) {
            typeDefDict[fieldName] = fieldType;
          }
          else {
            typeDefDict[fieldName] = [fieldType];
          }
        }
        else {
          // lookup the name
          var sub = false;
          for (var j = 0; j < hints.length; j++) {
            if (hints[j].type.toString() === fieldType.toString()) {
              sub = hints[j];
              break;
            }
          }
          if (sub) {
            var subResult = decodeTypeDefsRec(sub, hints);
            if (arrayLen === -1) {
            }
            else {
              typeDefDict[fieldName] = [subResult];
            }
          }
          else {
            // that.emit('error', 'Cannot find ' + fieldType + ' in decodeTypeDefs');
          }
        }
      }
      return typeDefDict;
    };

    return decodeTypeDefsRec(defs[0], defs);
  };

  function getTopics() {
    // var defer = $.Deferred();
    // var topicNameList = [];
    // var topicTypeList = [];
    // var detailItems = [];
    // var detailList = [];
    // var numberList = [];
    // var nameList = [];
    // var parentList = [];

    ros.getTopics(function (topicInfo) {
      // console.log(topicInfo);
      var promises = [];

      _.each(topicInfo.topics, function (topicName, index) {
        var topicType = topicInfo.types[index];

        var promise = getMessageDetailsAsync(
          topicType,
          function (result) {
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

      $.when.apply(null, promises).done(function () {
        // 整形する
        var fieldList = [];
        _.each(topicDetailMap, function (details, topicName) {
          console.log(details);
          var decoded = ros.decodeTypeDefs(details.detail);
          // var decoded = decodeTypeDefs(details.detail);

          topicId++;
          var parent = {
            id: topicId,
            topic: topicName,
            type: details.topicType,
            bandwidth: undefined,
            hz: undefined,
            value: undefined,
            parent: null,
            indent: 0,
            _collapsed: true,
          };
          fieldList.push(parent);

          var typeList = getFieldList(decoded, parent);
          fieldList = fieldList.concat(typeList);
        });

        // TODO: treeの開閉の状態をdataから取得して引き継ぐ

        // TODO: ソートの状態を引き継ぐ
        // 初回表示なら topicNameでソートする

        data = fieldList;
        console.log(fieldList);

        // draw grid
        dataView.beginUpdate();
        dataView.setItems(fieldList);
        dataView.endUpdate();

      });
    });

    // $.when(deferTopic).done(function (topicInfo) {
    //   var promises = [];
    //   $.each(topicInfo.types, function (index, topic_item) {
    //     var promise = getMessageDetailsAsync(topic_item,
    //       function (detailItem) {
    //         // console.log('detailItem');
    //         // console.log(detailItem);
    //         // console.log(topic_item);
    //         topicTypeList.push(topic_item);


    //         // _.each(detailItem, function (item, index) {
    //         //   detailItems.push({ name: item.fieldnames, type: item.fieldtypes });
    //         // });

    //         var decodedItem = ros.decodeTypeDefs(detailItem);
    //         // console.log(detailItem);
    //         // console.log(decodedItem);
    //         detailList.push(decodedItem);
    //         numberList.push(index);
    //       },
    //       function (message) {
    //         console.error('error at getMessageDetailsAsync().' + message);
    //       }
    //     );
    //     promises.push(promise);
    //   });

    //   $.when(promises).done(function () {
    //     // console.log(detailItems);
    //     $.each(numberList, function (numberIndex, number) {
    //       var number_change = parseInt(number, 10);
    //       nameList.push(topicNameList[number_change]);
    //     });

    //     // console.log(topicNameList);
    //     // console.log(nameList);
    //     // console.log(numberList);
    //     // console.log(topicTypeList);
    //     // console.log(detailItems);
    //     // console.log(detailList);
    //     getTopicData(topicNameList);
    //     getNumList(numberList);
    //     // console.log(data);
    //   });
    //   // grid = new Slick.Grid('#myGrid', data, columns);
    // });

  }


  // //TODO test
  // function getTopic() {
  //   var deferTopic = $.Deferred();
  //   var promises = [deferTopic.promise()];

  //   ros.getTopics(
  //     function (names) {
  //       // for (var j = 0; j < topic_info.topics.length; j++) {
  //       //   topicNameList.push(topic_info.topics[j]);
  //       // }
  //       console.log('---- Get topics ----');
  //       console.log(names);
  //       // names.sort();
  //       var topics = names.topics;
  //       var types = names.types;
  //       // _.each(topics, function (name, index) {
  //       // var promise = getTopicTypes(name);
  //       _.each(types, function (name, index) {
  //         console.log('---- Topic type ----');
  //         console.log(topics[index] + ' / ' + name);

  //         var promise = getTopicTypes(topics[index], name, topicTypeDetail);
  //         promises.push(promise);

  //         var promiseHzBw = getHzBw(topics[index], name, info);
  //         promises.push(promiseHzBw);

  //       });

  //       deferTopic.resolve();
  //       $.when.apply(null, promises).done(function () {

  //         // 非同期処理が全部終わったときの処理
  //         console.log('---- get topic end ----');
  //         // gridList(rosparamList);
  //         // });
  //       });
  //     });
  // }

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
        console.log('getMessageDetailsAsync success');
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

  function getTopicData(topicNameList) {
    for (var i = 0; i < topicNameList.length; i++) {
      var recordTopicData = topicNameList[i];
      // console.log(recordTopicData);
      // data.push({
      //   topic: recordTopicData,
      // });
      dataView.beginUpdate();
      dataView.addItem({
        topic: recordTopicData,
      });
      dataView.endUpdate();

    }
  }

  function getNumList(numberList) {
    for (var i = 0; i < numberList.length; i++) {
      var recordNumData = numberList[i];
      // console.log(recordNumData);
      data.push({
        type: recordNumData,
      });
    }
  }



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

  grid.onClick.subscribe(function gridClickhandler(e, args) {
    var $target = $(e.target);

    // event handler: toggle tree button
    if ($target.hasClass('toggle')) {
      toggleTree(e, args);
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
