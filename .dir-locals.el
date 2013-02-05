(
 (nil . ((eval . (setq default-directory (locate-dominating-file
					  buffer-file-name ".dir-locals.el")))
	 (eval . (set (make-local-variable 'my-project-path)
                      (file-name-directory
                       (let ((d (dir-locals-find-file ".")))
                         (if (stringp d) d (car d))))))
	 (eval . (progn
		   (setq savehist-additional-variables
			 '(search-ring regexp-search-ring)
			 savehist-file (expand-file-name ".savehist.el"
							 my-project-path))
		    (savehist-mode 1)))))
)
