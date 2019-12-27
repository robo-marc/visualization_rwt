
var nodeGrid;
var paramGrid;
var topicGrid;
var nodeList = [];
var paramList = [];
var topicList = [];

// subscribe topic
var ros = new ROSLIB.Ros();
// ros.install_config_button('config-button');
ros.autoConnect();

// NodeListのグリッド
var nodeColumns = [
  { id: 'node', name: 'Node', field: 'node', width: 300, minWidth: 20, maxWidth: 800 },
  { id: 'status', name: 'Status', field: 'status', width: 300, minWidth: 20, maxWidth: 800 },
];

function timer() {
  ros.getNodes(function (nodesData) {
    nodeList = [];
    var that = this;
    console.log(nodesData);
    if (nodesData.indexOf('/move_group') !== -1) {
      nodeList.push({
        node: '/move_group',
        status: 'Running',
      })
    }
    else {
      nodeList.push({
        node: '/move_group',
        status: 'Not running',
      })
    }

    console.log(nodeList);
    paramGrid = new Slick.Grid("#nodeGrid", nodeList, nodeColumns);

    setTimeout(function () {
      that.timer();
    }, 5000);
  })
}

// ParameterListのグリッド
var paramcolumns = [
  { id: 'param_name', name: 'Param name', field: 'param_name', width: 300, minWidth: 20, maxWidth: 800 },
  { id: 'found', name: 'Found on Parameter Server?', field: 'found', width: 300, minWidth: 20, maxWidth: 800 },
];

function timer2() {
  ros.getParams(function (params) {
    paramList = [];
    var that = this;
    console.log(params);
    if (params.indexOf('/robot_desctiption' && '/robot_description_semantic') !== -1) {
      paramList.push({
        param_name: '/robot_desctiption',
        found: 'yes',
      })
      paramList.push({
        param_name: '/robot_description_semantic',
        found: 'yes',
      })
    }
    else if (params.indexOf('/robot_desctiption') !== -1 && (params.indexOf('/robot_description_semantic') == -1)) {
      paramList.push({
        param_name: '/robot_desctiption',
        found: 'yes',
      })
      paramList.push({
        param_name: '/robot_description_semantic',
        found: 'no',
      })
    }
    else if (params.indexOf('/robot_desctiption') == -1 && (params.indexOf('/robot_description_semantic') !== -1)) {
      paramList.push({
        param_name: '/robot_desctiption',
        found: 'no',
      })
      paramList.push({
        param_name: '/robot_description_semantic',
        found: 'yes',
      })
    }
    else {
      paramList.push({
        param_name: '/robot_desctiption',
        found: 'no',
      })
      paramList.push({
        param_name: '/robot_description_semantic',
        found: 'no',
      })
    }

    console.log(paramList);
    paramGrid = new Slick.Grid("#paramGrid", paramList, paramcolumns);


    setTimeout(function () {
      that.timer2();
    }, 5000);
  })

}

//topicListのグリッド（rwt_topicから、グリッド部分を挿入）
var topicColumns = [
  { id: 'topic', name: 'Topic', field: 'topic', width: 200, minWidth: 20, maxWidth: 900, sortable: true },
  { id: 'type', name: 'Type', field: 'type', width: 200, minWidth: 20, maxWidth: 900, sortable: true },
  { id: 'bandwidth', name: 'Bandwidth', field: 'bandwidth', width: 100, minWidth: 20, maxWidth: 300, sortable: true },
  { id: 'hz', name: 'Hz', field: 'hz', width: 100, minWidth: 20, maxWidth: 900, sortable: true },
  { id: 'value', name: 'Value', field: 'value', width: 200, minWidth: 20, maxWidth: 900, sortable: true },
];

topicGrid = new Slick.Grid("#topicGrid", topicList, topicColumns);

timer();
timer2();
