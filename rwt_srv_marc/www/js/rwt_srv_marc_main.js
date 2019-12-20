
$(function () {

  // subscribe topic
  var ros = new ROSLIB.Ros();
  ros.install_config_button("config-button");
  var serviceMap = new Map();

  //テスト request

  function requestService() {
    ros.getSrvList(function (result) {
      var objList = [];
      $.each(result.message, function (index, msg) {
        var replace = msg.split('\'').join('\"');

        obj = JSON.parse(replace);
        objList.push(obj);
        // console.log(obj);
      })
      console.log(objList);
      selectService(objList);

      $("#service-form").submit(function (e) {
        e.preventDefault();
        var service_name = $("#service-select").val();
        if (sub) {
          console.log("unsubscribe");
          sub.unsubscribe();
        }
      });
    }, function (mes) {
      console.log(mes);
    });
  }

  // // ros events///////////////////////////////
  // ros.on("connection", function () {
  //   ros.getTopics(function (topic_info) {
  //     var topics = topic_info.topics;
  //     topics.sort();
  //     //console.log("20：" + topics);
  //     $("#topic-select").append(_.map(topics, function (topic) {
  //       return '<option value="' + topic + '">' + topic + "</option>";
  //     }).join("\n"));
  //     $("#topic-select").change();
  //   });
  // });

  // ros.on("close", function () {
  //   $("#topic-select").empty();
  // });
  // ///////////////////////////////////////////

  requestService();

  // ★グリッドアイテム取得/////////////////////////////////////////
  var grid;
  var columns = [
    { id: "tree", name: "Tree", field: "tree", width: 200, minWidth: 20, maxWidth: 300 },
    { id: "type", name: "Type", field: "type", width: 260, minWidth: 20, maxWidth: 900 },
    { id: "path", name: "Path", field: "path", width: 260, minWidth: 20, maxWidth: 300 },
    { id: "remove", name: "Remove", field: "remove", width: 100, minWidth: 20, maxWidth: 200 }
  ];
  var data = [];

  // for (var i = 0; i < 30; i++) {
  //   data[i] = {
  //     tree: "",
  //     type: "type" + i,
  //     path: "",
  //     remove: ""
  //   };
  // }

  grid = new Slick.Grid("#myGrid", data, columns);

  // serviceのリストを表示するリストボックス
  function selectService(objList) {
    // var serviceList = objList;
    var firstAddServiceStr = "";
    for (var i = 0; i < objList.length; i++) {
      var recordData = objList[i]
      for (var j = 0; j < recordData.length; j++) {
        serviceElement = recordData[j]
        var serviceStr = serviceElement.substring(0, serviceElement.indexOf("/"));
        var messageStr = serviceElement.substring(serviceElement.indexOf("/") + 1);

        if (serviceMap.has(serviceStr)) {
          serviceMap.get(serviceStr).push(messageStr);
        } else {
          if (0 == serviceMap.size) {
            firstAddServiceStr = serviceStr;
          }
          var mapValue = [messageStr];
          serviceMap.set(serviceStr, mapValue);
          $("#service-select").append('<option value="' + serviceStr + '">' + serviceStr + "</option>");
        }
      }
    }
    var messageList = serviceMap.get(firstAddServiceStr)
    for (var k = 0; k < messageList.length; k++) {
      $("#message-select").append('<option value="' + messageList[k] + '">' + messageList[k] + "</option>");
    }
  };

  $('#service-select').on('change', function () {
    var selectedValue = $("#service-select").val();
    $("#message-select").empty();
    messageList = serviceMap.get(selectedValue);
    for (var k = 0; k < messageList.length; k++) {
      $("#message-select").append('<option value="' + messageList[k] + '">' + messageList[k] + "</option>");
    }
  })

  $('#add-service-button').on('click', function () {
    var serviceName = $('#service-select').val();
    var messageName = $('#message-select').val();
    RequestData = []
    for (var i = 0; i < 1; i++) {
      RequestData[i] = {
        tree: "Request",
        type: serviceName + "/" + messageName,
        path: serviceName + "/" + messageName,
        remove: "",
      };
    }

    data.push(RequestData)

    for (var n = 0; n < data.length; n++) {
      var recordObj = data[n];
      grid = new Slick.Grid("#myGrid", recordObj, columns);
    }

    // grid = new Slick.Grid("#myGrid", data, columns);
  });
});



