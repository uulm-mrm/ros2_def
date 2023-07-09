# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = .
BUILDDIR      = _build

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile clean html_clean clean_tikz_figure_intermediates clean_tikz_figures figures

clean_tikz_figures: clean_tikz_figure_intermediates
	rm -f thesis/tikz_figures/*.png

clean_tikz_figure_intermediates:
	rm -f thesis/tikz_figures/*.pdf thesis/tikz_figures/*.log thesis/tikz_figures/*.aux

thesis/tikz_figures/%.png: thesis/tikz_figures/%.tex
	cd thesis/tikz_figures && pdflatex -shell-escape $*.tex > /dev/null

clean: clean_tikz_figures
	@$(SPHINXBUILD) -M clean "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

html_clean:
# Deliberately like that, to ensure ordering...
	$(MAKE) clean
	$(MAKE) html
	$(MAKE) clean_tikz_figure_intermediates

figures: thesis/tikz_figures/impl-problem_description-example_nodegraph.png \
	thesis/tikz_figures/nodegraph-example_reordering.png \
	thesis/tikz_figures/impl-problem_description-example_nodegraph.png \
	thesis/tikz_figures/nodegraph-example_parallel_nodes.png \
	thesis/tikz_figures/nodegraph-example_parallel_topics.png \
	thesis/tikz_figures/eval-service-sequence_orchestrator.png \
	thesis/tikz_figures/eval-service-sequence_before.png \
	thesis/tikz_figures/nodegraph-example_multiple_publishers.png \
	thesis/tikz_figures/nodegraph-example_service_calls.png \
	thesis/tikz_figures/impl-callbacks-custom_exec.png \
	thesis/tikz_figures/impl-callbacks-rcl.png \
	thesis/tikz_figures/impl-callbacks-orchestrator_design.png \
	thesis/tikz_figures/impl-topic_interception_before.png \
	thesis/tikz_figures/impl-topic_interception_after.png \
	thesis/tikz_figures/impl-example_cb_graph.png \
	thesis/tikz_figures/impl-reconfig_sequence.png

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile figures
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
