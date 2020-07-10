define target remote
  target extended-remote $arg0

  # Enable target power
  # monitor tpwr enable

  # Scan and attach to the first target
  monitor swdp_scan
  attach 1

  # print demangled symbols
  set print asm-demangle on

  set mem inaccessible-by-default off

  # set backtrace limit to not have infinite backtrace loops
  set backtrace limit 32

  # detect unhandled exceptions, hard faults and panics
  break DefaultHandler
  break HardFault
  break rust_begin_unwind

end