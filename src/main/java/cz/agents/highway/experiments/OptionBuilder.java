package cz.agents.highway.experiments;

import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;

/**
 * Simplify creating command line options
 * Created by wmatex on 27.11.14.
 */
public class OptionBuilder {
    private Options options;

    public OptionBuilder(Options options) {
        this.options = options;
    }

    /**
     * Adds one option to the set of options
     *
     * @param option      Option name. e.g. -name
     * @param description User-friendly description of the option
     * @param hasArg      Whether the option has argument or is boolean
     * @param required    Whether this option is required
     * @return
     */
    public OptionBuilder add(String option, String description, boolean hasArg, boolean required) {
        Option opt = org.apache.commons.cli.OptionBuilder
                .withDescription(description)
                .hasArg(hasArg)
                .isRequired(required)
                .create(option);
        options.addOption(opt);
        return this;
    }
}
