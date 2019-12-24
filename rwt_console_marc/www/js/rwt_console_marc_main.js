function requiredFieldValidator(value) {
  if (value === null || value === undefined || !value.length) {
    return { valid: false, msg: "This is a required field" };
  } else {
    return { valid: true, msg: null };
  }
}

var dataView;
var grid;
var data = [];
var columns = [
  { id: "#", name: "#", field: "#", sortable: true },
  { id: "Message", name: "Message", width: 240, field: "Message", sortable: true },
  { id: "Severity", name: "Severity", field: "Severity", sortable: true },
  { id: "Node", name: "Node", field: "Node", sortable: true },
  { id: "Stamp", name: "Stamp", field: "Stamp", width: 240, sortable: true },
  { id: "Topics", name: "Topics", field: "Topics", width: 240, sortable: true },
  { id: "Location", name: "Location", field: "Location", width: 240, sortable: true },
  { id: "Number", name: "Number", field: "Number", width: -1, maxWidth: -1, minWidth: -1, resizable: false, headerCssClass: 'hidden', sortable: true }
];

var options = {
  editable: true,
  enableAddRow: false,
  enableCellNavigation: true,
  asyncEditorLoading: false
};

function myFilter(item) {
  var percentCompleteThreshold = 0;
  var searchString = "";
  if (item["percentComplete"] < percentCompleteThreshold) {
    return false;
  }

  if (searchString !== "" && item["title"].indexOf(searchString) === -1) {
    return false;
  }

  if (item.parent !== null) {
    var parent = data[item.parent];

    while (parent) {
      if (parent._collapsed || (parent["percentComplete"] < percentCompleteThreshold) || (searchString !== "" && parent["title"].indexOf(searchString) === -1)) {
        return false;
      }
      parent = data[parent.parent];
    }
  }
  return true;
}

function percentCompleteSort(a, b) {
  return a["percentComplete"] - b["percentComplete"];
}

dataView = new Slick.Data.DataView({ inlineFilters: true });
grid = new Slick.Grid("#myGrid", dataView, columns, options);
var list = [];
var isSubscribe = 1;
var count = 1;
var isAsc = false;
$(function () {
  // subscribe topic
  var ros = new ROSLIB.Ros();
  ros.install_config_button("config-button");

  $('#pauseButton').show();
  $('#startButton').hide();

  function startDrawing() {
    var sub = null;
    sub = new ROSLIB.Topic({
      ros: ros,
      name: '/rosout_agg',
      messageType: 'rosgraph_msgs/Log'
    });
    sub.subscribe(function (msg) {
      if (isSubscribe === 1) {
        var intTime = msg.header.stamp.secs;
        var d = new Date(intTime * 1000);
        var year = d.getFullYear();
        var month = d.getMonth() + 1;
        var day = d.getDate();
        var hour = ('0' + d.getHours()).slice(-2);
        var min = ('0' + d.getMinutes()).slice(-2);
        var sec = ('0' + d.getSeconds()).slice(-2);

        var regular = ('0000000000' + msg.header.stamp.nsecs).slice(-9);
        var time = hour + ":" + min + ":" + sec + "." + regular + "(" + year + "-" + month + "-" + day + ")";
        var mes = msg.msg;
        var severiltyNumber = msg.level;
        var severilty = null;
        var node = msg.name;
        var stamp = time;
        var topics = msg.topics.join(',');
        var location = msg.file + ":" + msg.function + ":" + msg.line;

        if (msg.level === 1) {
          severilty = 'DEBUG';
        } else if (msg.level === 2) {
          severilty = 'INFO';
        } else if (msg.level === 4) {
          severilty = 'WARN';
        } else if (msg.level === 8) {
          severilty = 'ERROR';
        } else if (msg.level === 16) {
          severilty = 'FATAL';
        }

        var associationItem = {
          id: count,
          indent: 0,
          '#': '#' + count,
          Message: mes,
          SeveriltyNumber: severiltyNumber,
          Severity: severilty,
          Node: node,
          Stamp: stamp,
          RawTime: msg.header.stamp.secs + '.' + regular,
          Topics: topics,
          Location: location,
          Number: count
        };

        if (count >= 20001) {
          dataView.deleteItem(count - 20000);
        }

        list.push(associationItem);

        if (isAsc === false) {
          dataView.insertItem(0, associationItem);
          grid.invalidate();
        }

        if (isAsc === true) {
          dataView.insertItem(count, associationItem);
          grid.invalidate();
        }

        count++;

        var txt = list.length + ' messages';
        document.getElementById("NumberOfMessage").innerHTML = txt;
      }
    });
  }

  //グラフをクリアする
  $('#clearButton').click(function () {
    list.length = 0;
    data.length = 0;
    count = 1;
    dataView.setItems(data);
    dataView.setFilter(myFilter);
    grid.invalidate();
    document.getElementById("NumberOfMessage").innerHTML = '0 messages';
  });

  //CSVをダウンロード
  $('#downloadCSV').click(function () {
    var arr = [];
    var itemList = _.cloneDeep(list);
    _.each(itemList, function (value, index) {
      delete value.id;
      delete value['#'];
      delete value.Stamp;
      delete value.indent;
      delete value.Severity;
      arr.push(Object.keys(value).map(function (key) {
        return value[key];
      })
      );
    });
    var csvData = '';
    _.each(arr, function (value, index) {
      var row = value.join(';');
      csvData = row + '\n' + csvData;
    });
    var csvText = 'message;severity;node;stamp;topics;location\n' + csvData;
    var blob = new Blob([csvText], { "type": "text/plain" });
    if (window.navigator.msSaveBlob) {
      window.navigator.msSaveBlob(blob, "test.csv");
      window.navigator.msSaveOrOpenBlob(blob, "test.csv");
    } else {
      document.getElementById("downloadCSV").href = window.URL.createObjectURL(blob);
    }
  });


  //一時停止
  $('#pauseButton').click(function () {
    isSubscribe = 0;
    $('#pauseButton').hide();
    $('#startButton').show();
  });

  //再開ボタン
  $('#startButton').click(function () {
    isSubscribe = 1;
    $('#pauseButton').show();
    $('#startButton').hide();
  });

  //並び替え
  var sortcol;
  grid.onSort.subscribe(function (e, args) {
    sortcol = args.sortCol.field;

    if (sortcol === "#") {
      sortcol = "Number";
    }
    if (sortcol === "Number" || sortcol === "Stamp") {
      isAsc = args.sortAsc;
      dataView.sort(comparer, isAsc);
      grid.invalidateAllRows();
      grid.render();
    }
  });

  function comparer(a, b) {
    var x = a[sortcol], y = b[sortcol];
    return (x === y ? 0 : (x > y ? 1 : -1));
  }


  startDrawing();

});

// var sample = function () {
//   var indent = 0;
//   var parents = [];
//   var rowCount = 1;
//   // prepare the data
//   _.each(list, function (msg, index) {

//     var d = (data[index] = {});
//     var parent;

//     if (parents.length > 0) {
//       parent = parents[parents.length - 1];
//     } else {
//       parent = null;
//     }
//     d["id"] = "id_" + index;
//     d["indent"] = indent;
//     d["parent"] = parent;
//     d["#"] = '#' + rowCount;
//     d["Message"] = msg.Message;
//     d["Severity"] = severilty;
//     d["Node"] = msg.Node;
//     d["Stamp"] = msg.Stamp;
//     d["Topics"] = msg.Topics;
//     d["Location"] = msg.Location;

//     rowCount++;
//   });


//   // initialize the model
//   dataView.beginUpdate();
//   var sortData = data.slice().reverse();
//   dataView.setItems(sortData);
//   dataView.setFilter(myFilter);
//   dataView.endUpdate();


//   // initialize the grid
//   grid = new Slick.Grid("#myGrid", dataView, columns, options);

//   grid.onCellChange.subscribe(function (e, args) {
//     dataView.updateItem(args.item.id, args.item);
//   });

//   // wire up the search textbox to apply the filter to the model
//   $("#txtSearch").keyup(function (e) {
//     Slick.GlobalEditorLock.cancelCurrentEdit();

//     // clear on Esc
//     if (e.which === 27) {
//       this.value = "";
//     }

//     searchString = this.value;
//     dataView.refresh();
//   });
// };