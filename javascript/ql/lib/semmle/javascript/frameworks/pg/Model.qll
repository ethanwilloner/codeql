/** Generated model file */

private import javascript

private class Types extends ModelInput::TypeModelCsv {
  override predicate row(string row) {
    row =
      [
        "events;pg-cursor;", //
        "events;pg-promise/pg-subset.pg.IClient;", //
        "events;pg-promise/pg-subset.pg.IConnection;", //
        "events;pg-promise/pg-subset.pg.IPool;", //
        "events;pg.ClientBase;", //
        "events;pg.Events;", //
        "events;pg.Pool;", //
        "global.NodeJS.EventEmitter;events;", //
        "pg-cursor.Static;pg-cursor;", //
        "pg-cursor;pg-cursor.Static;Instance", //
        "pg-pool.Static;pg-pool;", //
        "pg-pool;pg-pool.Static;Instance", //
        "pg-pool;pg-pool;Member[addListener,on,once,prependListener,prependOnceListener].ReturnValue", //
        "pg-promise.IBaseProtocol;pg-promise.IConnected;", //
        "pg-promise.IBaseProtocol;pg-promise.IDatabase;", //
        "pg-promise.IBaseProtocol;pg-promise.ITask;", //
        "pg-promise.IBaseProtocol;pg-promise/typescript/pg-promise.IBaseProtocol;", //
        "pg-promise.IConnected;pg-promise.IDatabase;Member[connect].ReturnValue.Awaited", //
        "pg-promise.IConnected;pg-promise/typescript/pg-promise.IConnected;", //
        "pg-promise.IDatabase;pg-promise.IInitOptions;Member[extend].Argument[0]", //
        "pg-promise.IDatabase;pg-promise.IMain;ReturnValue", //
        "pg-promise.IDatabase;pg-promise/typescript/pg-promise.IDatabase;", //
        "pg-promise.IInitOptions;pg-promise.ILibConfig;Member[options]", //
        "pg-promise.IInitOptions;pg-promise/typescript/pg-promise.IInitOptions;", //
        "pg-promise.IInitOptions;pg-promise;Argument[0]", //
        "pg-promise.ILibConfig;pg-promise.IDatabase;Member[$config]", //
        "pg-promise.ILibConfig;pg-promise/typescript/pg-promise.ILibConfig;", //
        "pg-promise.IMain;pg-promise.ILibConfig;Member[pgp]", //
        "pg-promise.IMain;pg-promise/typescript/pg-promise.IMain;", //
        "pg-promise.IMain;pg-promise;ReturnValue", //
        "pg-promise.ITask;pg-promise.IBaseProtocol;Member[task,taskIf,tx,txIf].Argument[1].Argument[0]", //
        "pg-promise.ITask;pg-promise.IBaseProtocol;Member[task,taskIf,tx,txIf].WithArity[1].Argument[0].Argument[0]", //
        "pg-promise.ITask;pg-promise.IBaseProtocol;Member[taskIf].WithArity[2].Argument[0].Member[cnd].Argument[0]", //
        "pg-promise.ITask;pg-promise.IBaseProtocol;Member[txIf].WithArity[2].Argument[0].Member[cnd,reusable].Argument[0]", //
        "pg-promise.ITask;pg-promise/typescript/pg-promise.ITask;", //
        "pg-promise/pg-subset.pg.IClient;pg-promise.IMain;Argument[0].TypeVar[pg-promise/pg-subset.pg.IConnectionParameters.0]", //
        "pg-promise/pg-subset.pg.IClient;pg-promise.IMain;ReturnValue.TypeVar[pg-promise.IDatabase.1]", //
        "pg-promise/pg-subset.pg.IClient;pg-promise/pg-subset;Member[Client].Instance", //
        "pg-promise/pg-subset.pg.IClient;pg-promise;Argument[0].TypeVar[pg-promise.IInitOptions.1]", //
        "pg-promise/pg-subset.pg.IConnection;pg-promise/pg-subset.pg.IClient;Member[connection]", //
        "pg-promise/pg-subset.pg.IPool;pg-promise.IDatabase;Member[$pool]", //
        "pg.Client;pg-pool.Static;Instance.TypeVar[pg-pool.0]", //
        "pg.Client;pg-promise/pg-subset.pg.IClient;", //
        "pg.Client;pg.ClientStatic;Instance", //
        "pg.Client;pg.Events;Member[addListener,on,once,prependListener,prependOnceListener].Argument[1].Argument[1]", //
        "pg.ClientBase;pg.Client;", //
        "pg.ClientBase;pg.PoolClient;", //
        "pg.ClientStatic;pg;Member[Client]", //
        "pg.Connection;pg-promise/pg-subset.pg.IConnection;", //
        "pg.Events;pg.Events;Member[addListener,on,once,prependListener,prependOnceListener].ReturnValue", //
        "pg.Events;pg.EventsStatic;Instance", //
        "pg.EventsStatic;pg;Member[Events]", //
        "pg.Pool;pg-pool;", //
        "pg.Pool;pg-promise/pg-subset.pg.IPool;", //
        "pg.Pool;pg.Pool;Member[addListener,on,once,prependListener,prependOnceListener].ReturnValue", //
        "pg.Pool;pg.PoolStatic;Instance", //
        "pg.PoolClient;pg-pool;Member[addListener,on,once,prependListener,prependOnceListener].WithArity[2].WithStringArgument[0=acquire,0=connect,0=remove].Argument[1].Argument[0]", //
        "pg.PoolClient;pg-pool;Member[addListener,on,once,prependListener,prependOnceListener].WithArity[2].WithStringArgument[0=error].Argument[1].Argument[1]", //
        "pg.PoolClient;pg-pool;Member[connect].Argument[0].Argument[1]", //
        "pg.PoolClient;pg-pool;Member[connect].WithArity[0].ReturnValue.Awaited", //
        "pg.PoolClient;pg.Pool;Member[addListener,on,once,prependListener,prependOnceListener].WithArity[2].WithStringArgument[0=acquire,0=connect,0=remove].Argument[1].Argument[0]", //
        "pg.PoolClient;pg.Pool;Member[addListener,on,once,prependListener,prependOnceListener].WithArity[2].WithStringArgument[0=error].Argument[1].Argument[1]", //
        "pg.PoolClient;pg.Pool;Member[connect].Argument[0].Argument[1]", //
        "pg.PoolClient;pg.Pool;Member[connect].WithArity[0].ReturnValue.Awaited", //
        "pg.PoolStatic;pg;Member[Pool]", //
      ]
  }
}

private class Summaries extends ModelInput::SummaryModelCsv {
  override predicate row(string row) {
    row =
      [
        "global.NodeJS.EventEmitter;;;Member[addListener,off,on,once,prependListener,prependOnceListener,removeAllListeners,removeListener,setMaxListeners].ReturnValue;type", //
        "pg-pool;;;Member[addListener,on,once,prependListener,prependOnceListener].ReturnValue;type", //
        "pg.ClientBase;;;Member[addListener,on,once,prependListener,prependOnceListener].ReturnValue;type", //
        "pg.Events;;;Member[addListener,on,once,prependListener,prependOnceListener].ReturnValue;type", //
        "pg.Pool;;;Member[addListener,on,once,prependListener,prependOnceListener].ReturnValue;type", //
      ]
  }
}

private class TypeVariables extends ModelInput::TypeVariableModelCsv {
  override predicate row(string row) {
    row =
      [
        "pg-pool.0;Member[Client].TypeVar[pg-pool.ClientLikeCtr.0]", //
        "pg-pool.0;Member[addListener,on,once,prependListener,prependOnceListener].WithArity[2].WithStringArgument[0=acquire,0=connect,0=remove].Argument[1].Argument[0]", //
        "pg-pool.0;Member[addListener,on,once,prependListener,prependOnceListener].WithArity[2].WithStringArgument[0=error].Argument[1].Argument[1]", //
        "pg-pool.0;Member[connect].Argument[0].Argument[1]", //
        "pg-pool.0;Member[connect].WithArity[0].ReturnValue.Awaited", //
        "pg-pool.ClientLikeCtr.0;Instance", //
        "pg-promise.IConnected.1;Member[client]", //
        "pg-promise.IConnectionOptions.0;Member[onLost].Argument[1].TypeVar[pg-promise.ILostContext.0]", //
        "pg-promise.IDatabase.1;Member[$cn].TypeVar[pg-promise/pg-subset.pg.IConnectionParameters.0]", //
        "pg-promise.IDatabase.1;Member[$config].TypeVar[pg-promise.ILibConfig.1]", //
        "pg-promise.IDatabase.1;Member[connect].Argument[0].TypeVar[pg-promise.IConnectionOptions.0]", //
        "pg-promise.IDatabase.1;Member[connect].ReturnValue.Awaited.TypeVar[pg-promise.IConnected.1]", //
        "pg-promise.IEventContext.0;Member[client]", //
        "pg-promise.IInitOptions.1;Member[connect,disconnect].Argument[0]", //
        "pg-promise.IInitOptions.1;Member[error].Argument[1].TypeVar[pg-promise.IEventContext.0]", //
        "pg-promise.IInitOptions.1;Member[extend].Argument[0].TypeVar[pg-promise.IDatabase.1]", //
        "pg-promise.IInitOptions.1;Member[query,task,transact].Argument[0].TypeVar[pg-promise.IEventContext.0]", //
        "pg-promise.IInitOptions.1;Member[receive].Argument[2].TypeVar[pg-promise.IEventContext.0]", //
        "pg-promise.ILibConfig.1;Member[options].TypeVar[pg-promise.IInitOptions.1]", //
        "pg-promise.ILibConfig.1;Member[pgp].TypeVar[pg-promise.IMain.1]", //
        "pg-promise.ILostContext.0;Member[client]", //
        "pg-promise/pg-promise.XPromise.0;Awaited", //
        "pg-promise/pg-subset.pg.IConnectionParameters.0;Member[Client].Instance", //
      ]
  }
}
