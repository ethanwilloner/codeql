/** Generated model file */

private import javascript

private class Types extends ModelInput::TypeModelCsv {
  override predicate row(string row) {
    row =
      [
        "mssql.ConnectionPool;mssql.ConnectionPool;Member[connect].WithArity[0].ReturnValue.Awaited", //
        "mssql.ConnectionPool;mssql.ConnectionPoolStatic;Instance", //
        "mssql.ConnectionPool;mssql/msnodesqlv8;Member[connect].ReturnValue.Awaited", //
        "mssql.ConnectionPool;mssql/msnodesqlv8;Member[pool]", //
        "mssql.ConnectionPool;mssql;Member[connect].ReturnValue.Awaited", //
        "mssql.ConnectionPool;mssql;Member[pool]", //
        "mssql.ConnectionPoolStatic;mssql/msnodesqlv8;Member[ConnectionPool]", //
        "mssql.ConnectionPoolStatic;mssql;Member[ConnectionPool]", //
        "mssql.PreparedStatement;mssql.PreparedStatement;Member[input,output].ReturnValue", //
        "mssql.PreparedStatement;mssql.PreparedStatement;Member[prepare].WithArity[0,1,2].ReturnValue", //
        "mssql.PreparedStatement;mssql.PreparedStatement;Member[unprepare].WithArity[1].ReturnValue", //
        "mssql.PreparedStatement;mssql.PreparedStatementStatic;Instance", //
        "mssql.PreparedStatement;mssql.Request;Member[pstatement]", //
        "mssql.PreparedStatementStatic;mssql/msnodesqlv8;Member[PreparedStatement]", //
        "mssql.PreparedStatementStatic;mssql;Member[PreparedStatement]", //
        "mssql.Request;mssql.ConnectionPool;Member[request].ReturnValue", //
        "mssql.Request;mssql.PreparedStatement;Member[execute].WithArity[2].ReturnValue", //
        "mssql.Request;mssql.Request;Member[input,output,replaceInput].ReturnValue", //
        "mssql.Request;mssql.Request;Member[replaceOutput].ReturnValue", //
        "mssql.Request;mssql.RequestStatic;Instance", //
        "mssql.Request;mssql.Transaction;Member[request].ReturnValue", //
        "mssql.RequestStatic;mssql/msnodesqlv8;Member[Request]", //
        "mssql.RequestStatic;mssql;Member[Request]", //
        "mssql.Transaction;mssql.ConnectionPool;Member[transaction].ReturnValue", //
        "mssql.Transaction;mssql.PreparedStatement;Member[transaction]", //
        "mssql.Transaction;mssql.Request;Member[transaction]", //
        "mssql.Transaction;mssql.Transaction;Member[begin].WithArity[0,1,2].ReturnValue", //
        "mssql.Transaction;mssql.Transaction;Member[begin].WithArity[0,1].ReturnValue.Awaited", //
        "mssql.Transaction;mssql.TransactionStatic;Instance", //
        "mssql.TransactionStatic;mssql/msnodesqlv8;Member[Transaction]", //
        "mssql.TransactionStatic;mssql;Member[Transaction]", //
        "mssql.config;mssql.ConnectionPoolStatic;WithArity[1,2].Argument[0]", //
        "mssql.config;mssql/msnodesqlv8;Member[connect].Argument[0]", //
        "mssql.config;mssql;Member[connect].Argument[0]", //
      ]
  }
}
