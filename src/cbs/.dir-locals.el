;;; Directory Local Variables
;;; For more information see (info "(emacs) Directory Variables")

((c++-mode . (
  (flycheck-checker . c/c++-gcc)
  (flycheck-clang-standard-library . "libc++")
  (flycheck-clang-include-path . ())
  (flycheck-gcc-include-path . ("/usr/local/Cellar/boost/1.76.0/include/boost" "../../inc"  "/Applications/CPLEX_Studio128/cplex/include" "/Applications/CPLEX_Studio128/concert/include"))
  (flycheck-clang-language-standard . "c++14")
  (flycheck-gcc-language-standard . "c++14")
  )

           )

 (nil . (
         (eval . (add-to-list 'auto-mode-alist '("\\.h\\'" . c++-mode)))
         ))

 )




