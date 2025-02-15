/** Generated model file */

private import javascript

private class Types extends ModelInput::TypeModelCsv {
  override predicate row(string row) {
    row =
      [
        "mysql.Connection;mysql.Pool;Member[on,addListener].WithStringArgument[0=acquire,0=connection,0=release].Argument[1].Argument[0]", //
        "mysql.Connection;mysql.PoolConnection;", //
        "mysql.Connection;mysql.Query;Member[RowDataPacket].Argument[2]", //
        "mysql.Connection;mysql;Member[createConnection].ReturnValue", //
        "mysql.ConnectionConfig;mysql.Connection;Member[config]", //
        "mysql.ConnectionConfig;mysql.PoolConfig;", //
        "mysql.ConnectionConfig;mysql;Member[createConnection].Argument[0]", //
        "mysql.ConnectionOptions;mysql.Connection;Member[changeUser].WithArity[1,2].Argument[0]", //
        "mysql.ConnectionOptions;mysql.ConnectionConfig;", //
        "mysql.Pool;mysql.PoolCluster;Member[of].ReturnValue", //
        "mysql.Pool;mysql;Member[createPool].ReturnValue", //
        "mysql.PoolCluster;mysql;Member[createPoolCluster].ReturnValue", //
        "mysql.PoolConfig;mysql.PoolCluster;Member[add].Argument[1]", //
        "mysql.PoolConfig;mysql.PoolCluster;Member[add].WithArity[1].Argument[0]", //
        "mysql.PoolConfig;mysql;Member[createPool].Argument[0]", //
        "mysql.PoolConnection;mysql.Pool;Member[acquireConnection].Argument[0]", //
        "mysql.PoolConnection;mysql.Pool;Member[acquireConnection].Argument[1].Argument[1]", //
        "mysql.PoolConnection;mysql.Pool;Member[getConnection].Argument[0].Argument[1]", //
        "mysql.PoolConnection;mysql.PoolCluster;Member[getConnection].Argument[1,2].Argument[1]", //
        "mysql.PoolConnection;mysql.PoolCluster;Member[getConnection].WithArity[1].Argument[0].Argument[1]", //
        "mysql.Query;mysql.Query;Member[on].ReturnValue", //
        "mysql.Query;mysql.QueryFunction;ReturnValue", //
        "mysql.Query;mysql.QueryFunction;WithArity[1].Argument[0]", //
        "mysql.QueryFunction;mysql.Connection;Member[createQuery,query]", //
        "mysql.QueryFunction;mysql.Pool;Member[query]", //
        "mysql2.Connection;mysql2.PoolConnection;", //
        "mysql2.Connection;mysql2.authPlugins;Argument[0].Member[connection]", //
        "mysql2.Connection;mysql2;Member[createConnection].ReturnValue", //
        "mysql2.ConnectionOptions;mysql2.PoolOptions;", //
        "mysql2.ConnectionOptions;mysql2/promise.Connection;Member[changeUser].Argument[0]", //
        "mysql2.ConnectionOptions;mysql2/promise.Connection;Member[config]", //
        "mysql2.ConnectionOptions;mysql2/promise;Member[createConnection].Argument[0]", //
        "mysql2.ConnectionOptions;mysql2;Member[createConnection].Argument[0]", //
        "mysql2.Pool;mysql2.Pool;Member[addListener,on,once,prependListener,prependOnceListener].ReturnValue", //
        "mysql2.Pool;mysql2;Member[createPool].ReturnValue", //
        "mysql2.PoolConnection;mysql2.Pool;Member[addListener,on,once,prependListener,prependOnceListener].WithArity[2].WithStringArgument[0=acquire,0=connection,0=release].Argument[1].Argument[0]", //
        "mysql2.PoolConnection;mysql2.Pool;Member[getConnection].Argument[0].Argument[1]", //
        "mysql2.PoolOptions;mysql2/promise;Member[createPool].Argument[0]", //
        "mysql2.PoolOptions;mysql2;Member[createPool].Argument[0]", //
        "mysql2.authPlugins;mysql2.ConnectionOptions;Member[authPlugins].AnyMember", //
        "mysql2/promise.Connection;mysql2.Connection;Member[promise].ReturnValue", //
        "mysql2/promise.Connection;mysql2/promise.PoolConnection;", //
        "mysql2/promise.Connection;mysql2/promise;Member[createConnectionPromise].ReturnValue.Awaited", //
        "mysql2/promise.Connection;mysql2/promise;Member[createConnection].ReturnValue.Awaited", //
        "mysql2/promise.Connection;mysql2;Member[createConnectionPromise].ReturnValue.Awaited", //
        "mysql2/promise.Pool;mysql2.Pool;Member[promise].ReturnValue", //
        "mysql2/promise.Pool;mysql2/promise.Pool;Member[addListener,on,once,prependListener,prependOnceListener].ReturnValue", //
        "mysql2/promise.Pool;mysql2/promise;Member[createPool].ReturnValue", //
        "mysql2/promise.PoolConnection;mysql2.PoolConnection;Member[promise].ReturnValue", //
        "mysql2/promise.PoolConnection;mysql2/promise.Pool;Member[addListener,on,once,prependListener,prependOnceListener].WithArity[2].WithStringArgument[0=acquire,0=connection,0=release].Argument[1].Argument[0]", //
        "mysql2/promise.PoolConnection;mysql2/promise.Pool;Member[getConnection].ReturnValue.Awaited", //
        "mysql2/typings/mysql.Connection;mysql2.Connection;", //
        "mysql2/typings/mysql.Connection;mysql2.Pool;", //
        "mysql2/typings/mysql.PoolConnection;mysql2.PoolConnection;", //
        "mysql2/typings/mysql/lib/Connection;mysql2/typings/mysql.Connection;", //
        "mysql2/typings/mysql/lib/Connection;mysql2/typings/mysql/lib/PoolConnection;", //
        "mysql2/typings/mysql/lib/PoolConnection;mysql2/typings/mysql.PoolConnection;", //
      ]
  }
}

private class Summaries extends ModelInput::SummaryModelCsv {
  override predicate row(string row) {
    row =
      [
        "mysql2.Pool;;;Member[addListener,on,once,prependListener,prependOnceListener].ReturnValue;type", //
        "mysql2/promise.Pool;;;Member[addListener,on,once,prependListener,prependOnceListener].ReturnValue;type", //
        "mysql2/typings/mysql/lib/Connection;;;Member[addListener,on,once,prependListener,prependOnceListener].ReturnValue;type", //
      ]
  }
}
