var ros = new ROSLIB.Ros();
ros.install_config_button("config-button");
// ros.autoConnect();

var checkboxSelector;

var grid;
var data = [];

checkboxSelector = new Slick.CheckboxSelectColumn({
  cssClass: 'slick-cell-checkboxsel'
});

// ★グリッドアイテム取得/////////////////////////////////////////
var columns = [
  checkboxSelector.getColumnDefinition(),
  { id: 'topic', name: 'Topic', field: 'topic', width: 200, minWidth: 20, maxWidth: 300, sortable: true },
  { id: 'type', name: 'Type', field: 'type', width: 260, minWidth: 20, maxWidth: 900, sortable: true },
  { id: 'bandwidth', name: 'Bandwidth', field: 'bandwidth', width: 260, minWidth: 20, maxWidth: 300, sortable: true },
  { id: 'hz', name: 'Hz', field: 'hz', width: 260, minWidth: 20, maxWidth: 300, sortable: true },
  { id: 'value', name: 'Value', field: 'value', width: 260, minWidth: 20, maxWidth: 300, sortable: true },
];

// grid = new Slick.Grid("#myGrid", data, columns);

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
    };

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

      console.log(topicNameList);
      console.log(nameList);
      console.log(numberList);
      console.log(topicTypeList);
      console.log(detailItems);
      console.log(detailList);
      TopicData(topicNameList);
      NumList(numberList);
      console.log(data);
    });
    grid = new Slick.Grid("#myGrid", data, columns);
  });

});

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

function TopicData(topicNameList) {
  for (var i = 0; i < topicNameList.length; i++) {
    var recordTopicData = topicNameList[i];
    console.log(recordTopicData);

    data.push({
      topic: recordTopicData,
    });
  }
}

function NumList(numberList) {
  for (var i = 0; i < numberList.length; i++) {
    var recordNumData = numberList[i];
    console.log(recordNumData);
    data.push({
      type: recordNumData,
    });
  }
}

