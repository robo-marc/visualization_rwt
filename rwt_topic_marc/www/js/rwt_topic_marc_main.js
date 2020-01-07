// import { prependListener } from "cluster";

$(function () {

  var ros = new ROSLIB.Ros();
  // ros.autoConnect();

  var checkboxSelector;

  var grid;
  var data = [];
  var topicTypeDetail = {};
  var info = {};

  checkboxSelector = new Slick.CheckboxSelectColumn({
    cssClass: 'slick-cell-checkboxsel'
  });

  // ★グリッドアイテム取得/////////////////////////////////////////
  var columns = [
    checkboxSelector.getColumnDefinition(),
    { id: 'topic', name: 'Topic', field: 'topic', width: 160, minWidth: 20, maxWidth: 300, sortable: true },
    { id: 'type', name: 'Type', field: 'type', width: 260, minWidth: 20, maxWidth: 900, sortable: true },
    { id: 'bandwidth', name: 'Bandwidth', field: 'bandwidth', width: 120, minWidth: 20, maxWidth: 300, sortable: true },
    { id: 'hz', name: 'Hz', field: 'hz', width: 120, minWidth: 20, maxWidth: 300, sortable: true },
    { id: 'value', name: 'Value', field: 'value', width: 260, minWidth: 20, maxWidth: 300, sortable: true },
  ];

  $(document).ready(function aiuo() {
    var deferTopic = $.Deferred();
    var topicNameList = [];
    var topicTypeList = [];
    var detailItems = [];
    var detailList = [];
    var numberList = [];
    var nameList = [];

    ros.getTopics(function (topic_info) {
      for (var j = 0; j < topic_info.topics.length; j++) {
        topicNameList.push(topic_info.topics[j]);

      }

      deferTopic.resolve(topic_info);
    });

    $.when(deferTopic).done(function (topic_info) {
      var promises = [];
      $.each(topic_info.types, function (index, topic_item) {
        var promise = getMessageDetailsAsync(topic_item,
          function (detailItem) {

            topicTypeList.push(topic_item);
            detailItems.push(detailItem);
            var decodedItem = ros.decodeTypeDefs(detailItem);
            detailList.push(decodedItem);
            numberList.push(index);
          },
          function (message) {
            console.error('error at getMessageDetailsAsync().' + message);
          }
        );
        promises.push(promise);
      });

      $.when(promises).done(function () {
        $.each(numberList, function (numberIndex, number) {
          var number_change = parseInt(number, 10);
          nameList.push(topicNameList[number_change]);
        });

        // console.log(topicNameList);
        // console.log(nameList);
        // console.log(numberList);
        // console.log(topicTypeList);
        // console.log(detailItems);
        // console.log(detailList);
        // TopicData(topicNameList);
        NumList(numberList);
        // console.log(data);
      });
    });

  });
  // initialize screen
  function initScreen() {
    // common
    ros.autoConnect();
  }

  //TODO test
  function getTopic() {
    var deferTopic = $.Deferred();
    var promises = [deferTopic.promise()];

    ros.getTopics(
      function (names) {
        // for (var j = 0; j < topic_info.topics.length; j++) {
        //   topicNameList.push(topic_info.topics[j]);
        // }
        console.log('---- Get topics ----');
        console.log(names);
        // names.sort();
        var topics = names.topics;
        var types = names.types;
        // _.each(topics, function (name, index) {
        // var promise = getTopicTypes(name);
        _.each(types, function (name, index) {
          console.log('---- Topic type ----');
          console.log(topics[index] + ' / ' + name);

          var promise = getTopicTypes(topics[index], name, topicTypeDetail);
          promises.push(promise);

          var promiseHzBw = getHzBw(topics[index], name, info);
          promises.push(promiseHzBw);
        });

        deferTopic.resolve();
        $.when.apply(null, promises).done(function () {

          // 非同期処理が全部終わったときの処理
          console.log('---- get topic end ----');
          // gridList(rosparamList);
          // });

          grid = new Slick.Grid('#myGrid', data, columns);
        });
      });
  }

  // get Hz Bw
  function getHzBw(topic, type, info) {
    var defer = $.Deferred();

    ros.getTopicInfo(topic, type, function (value) {
      console.log('---- Hz Bw ----');
      console.log(topic + '  /  ' + type);
      console.log(value);
      info = value;
      defer.resolve();

      var bandwidth = value.bw;
      var hz = value.hz;

      data.push({
        topic: topic,
        type: type,
        bandwidth: bandwidth,
        hz: hz,
        value: ''
      })
    });
    return defer.promise();
  }

  // get topic type
  function getTopicTypes(topic, type, topicTypeDetail) {
    var defer = $.Deferred();

    // ros.getTopicType(name, function (topicType) {
    ros.getMessageDetails(type,
      function (details) {
        topicTypeDetail = details;
        console.log('---- Message detail ----');
        console.log(topic + '  /  ' + type);
        console.log(details);

        // TODO Tree構造を作る(action参考？)

        defer.resolve();
      },
      function (message) {
        failedCallback(message);
        defer.resolve();
      }
    );

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
      console.log(topic + '  /  ' + type);
      console.log(message);
      console.log(message.data);

      defer.resolve();
    });

    return defer.resolve();
  }

  function getMessageDetailsAsync(message, callback, failedCallback) {

    var defer = $.Deferred();

    ros.getMessageDetails(message,
      function (result) {
        callback(result);
        defer.resolve();
      },
      function (message) {
        failedCallback(message);
        defer.resolve();
      }
    );

    return defer.promise();
  }

  function NumList(numberList) {
    for (var i = 0; i < numberList.length; i++) {
      var recordNumData = numberList[i];
      // console.log(recordNumData);
      data.push({
        type: recordNumData,
      });
    }
  }
  initScreen();
  getTopic();
});
