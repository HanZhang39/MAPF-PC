
;;; Directory Local Variables
;;; For more information see (info "(emacs) Directory Variables")

((c++-mode . (
              (flycheck-checker . c/c++-gcc)
              (flycheck-clang-standard-library . "libc++")
              (flycheck-gcc-include-path . ("/usr/local/Cellar/boost/1.66.0/include/boost" "../src"))
              (flycheck-clang-language-standard . "c++14")
              (flycheck-gcc-language-standard . "c++14")
              )

           )

 (nil . (
         (eval . (add-to-list 'auto-mode-alist '("\\.h\\'" . c++-mode)))
         ))

 )



